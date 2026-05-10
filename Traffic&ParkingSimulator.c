/*group members:Saliha Noor, Sajal Ishtiaq
rollno.s: 24i-3066.24i-3041
 gcc i243066_i243041_OS_Project -o a -lraylib -lpthread -lm -lGL -ldl -lrt -lX11*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <semaphore.h>
#include <signal.h>
#include <time.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include "raylib.h"

/* 
   CONFIGURATION
*/
#define NUM_VEHICLES       15
#define PARKING_SPOTS      10
#define PARKING_QUEUE_MAX   5
#define GREEN_DURATION      6
#define RED_DURATURE        4
#define CROSSING_TIME_SLOW  3
#define CROSSING_TIME_FAST  2
#define SPAWN_DELAY_MIN     2000
#define SPAWN_DELAY_MAX     3500
#define LOG_MSG_LEN        100

#define TYPE_AMBULANCE  0
#define TYPE_FIRETRUCK  1
#define TYPE_BUS        2
#define TYPE_CAR        3
#define TYPE_BIKE       4
#define TYPE_TRACTOR    5

#define PRIORITY_EMERGENCY  1
#define PRIORITY_BUS        2
#define PRIORITY_NORMAL     3

#define DIR_STRAIGHT  0
#define DIR_LEFT      1
#define DIR_RIGHT     2

#define INTERSECTION_F10  0
#define INTERSECTION_F11  1

#define MSG_EMERGENCY_COMING  "EMERGENCY_COMING"
#define MSG_CLEAR_DONE        "CLEAR_DONE"
#define MSG_SHUTDOWN          "SHUTDOWN"
#define MSG_VEHICLE_PASSING   "VEHICLE_PASSING"

#define RED_TXT    "\033[1;31m"
#define GRN_TXT    "\033[1;32m"
#define YLW_TXT    "\033[1;33m"
#define BLU_TXT    "\033[1;34m"
#define MAG_TXT    "\033[1;35m"
#define CYN_TXT    "\033[1;36m"
#define RST_TXT    "\033[0m"

/* 
   VEHICLE STRUCT
*/
typedef struct {
    int     id;
    int     type;
    int     priority;
    char    type_name[16];
    char    origin[8];
    char    destination[8];
    int     direction;
    int     intersection;
    time_t  arrival_time;
    int     will_park;
    int     parked_slot;
    float   wait_time;
} Vehicle;

/*SYNCHRONISATION Globals*/

pthread_mutex_t mutex_f10          = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t  cond_f10_green     = PTHREAD_COND_INITIALIZER;
pthread_cond_t  cond_f10_bus       = PTHREAD_COND_INITIALIZER;  /* BUS priority lane */
int             light_f10_green    = 1;
int             f10_crossing_count = 0;
int             f10_crossing_dir   = 0; /* bitmask: bit0=NS axis, bit1=EW axis (non-conflicting) */

pthread_mutex_t mutex_f11          = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t  cond_f11_green     = PTHREAD_COND_INITIALIZER;
pthread_cond_t  cond_f11_bus       = PTHREAD_COND_INITIALIZER;  /* BUS priority lane */
int             light_f11_green    = 1;
int             f11_crossing_count = 0;
int             f11_crossing_dir   = 0; /* bitmask: bit0=NS axis, bit1=EW axis (non-conflicting) */

volatile int    emergency_active   = 0;
pthread_mutex_t mutex_emergency    = PTHREAD_MUTEX_INITIALIZER;

sem_t sem_spots_f10;
sem_t sem_queue_f10;
sem_t sem_spots_f11;
sem_t sem_queue_f11;

volatile sig_atomic_t running = 1;

int pipe_f10_to_f11[2];
int pipe_f11_to_f10[2];

/* SCREEN LAYOUT :
   Using 1340x740 to respect OS taskbar and window borders */
#define SIM_AREA_W    980
#define RIGHT_PANEL_W 360
#define SCREEN_W     (SIM_AREA_W + RIGHT_PANEL_W)
#define SCREEN_H      740

/* Road layout — horizontal main road at vertical centre */
#define ROAD_Y        368        /* Centre of horizontal road */
#define ROAD_HW        28        /* Half-width of each road arm */
#define LANE_W         13        /* Single lane width */

/* Intersection centres — scaled for wider SIM_AREA_W (980) */
#define F10_CX  240
#define F10_CY  ROAD_Y
#define F11_CX  740
#define F11_CY  ROAD_Y

/* Parking lots — inside the population districts below the road */
#define PK_F10_X   30
#define PK_F10_Y   512
#define PK_F11_X   510
#define PK_F11_Y   512

/* Population district bounding boxes */
#define DIST_F10_X    0
#define DIST_F10_Y  410
#define DIST_F10_W  490
#define DIST_F10_H  330

#define DIST_F11_X  490
#define DIST_F11_Y  410
#define DIST_F11_W  490
#define DIST_F11_H  330

/* Right panel */
#define RP_X (SIM_AREA_W + 6)

/*   ROAD-ONLY NAVIGATION WAYPOINTS
   All vehicle movement must follow the actual road network:
     • Vertical roads at x = F10_CX  and  x = F11_CX
     • Horizontal road  at y = ROAD_Y
   No vehicle ever moves diagonally or off-road.
   Waypoint sequences are computed here so every call site is clean.

   Lane offsets (right-hand traffic):
     North-bound (going south→north):  x = cx - LANE_W   (left lane)
     South-bound (going north→south):  x = cx + LANE_W   (right lane)
     East-bound  (going west→east):    y = ROAD_Y - LANE_W
     West-bound  (going east→west):    y = ROAD_Y + LANE_W
*/
#define ROAD_LANE_INBOUND   10    /* right lane : vehicle approaching intersection */
#define ROAD_LANE_OUTBOUND  10    /* left lane  : vehicle leaving intersection     */

/* Spawn edge positions — far enough off screen to slide in smoothly */
#define SPAWN_MARGIN  220

/*   VISUALISATION STATE*/
#define MAX_VIS_VEHICLES  NUM_VEHICLES
#define MAX_LOG_ENTRIES   120
#define MAX_PMSG          40

typedef struct {
    int    id, type, priority, intersection, state, active;
    char   type_name[16], origin[8], destination[8];
    float  x, y, tx, ty, progress, flash, angle;
    time_t arrival_time;
    int    will_park, parked_slot;
    /* Road-waypoint queue — up to 8 road corners per journey */
    float  wp_x[8], wp_y[8];
    int    wp_count, wp_idx;
} VisVehicle;

typedef struct {
    char  msg[LOG_MSG_LEN];
    Color col;
} LogEntry;

typedef struct {
    char  txt[80];
    float progress, alpha;
    int   dir, alive;
} PMsg;

/* OS Concept floating label */
typedef struct {
    char  label[48];
    char  concept[32];
    float x, y, alpha, life;
    Color col;
    int   active;
} OSLabel;

#define MAX_OS_LABELS 16
static OSLabel os_labels[MAX_OS_LABELS];
static int     os_label_idx = 0;

static pthread_mutex_t vis_mutex = PTHREAD_MUTEX_INITIALIZER;
static VisVehicle vis_vehicles[MAX_VIS_VEHICLES];
static int vis_veh_count = 0;
static LogEntry vis_log[MAX_LOG_ENTRIES];
static int vis_log_head = 0, vis_log_count = 0;
static int vis_light_f10 = 1, vis_light_f11 = 1, vis_emergency = 0;
static int vis_park_f10 = 0, vis_park_f11 = 0;
static int vis_parkq_f10 = 0, vis_parkq_f11 = 0;
static int vis_park_slots_f10[PARKING_SPOTS];
static int vis_park_slots_f11[PARKING_SPOTS];
static int vis_stat_crossed[2] = {0,0};
static int vis_stat_parked[2] = {0,0};
static int vis_stat_emg[2] = {0,0};
static int vis_stat_done = 0;
static PMsg vis_pmsgs[MAX_PMSG];
static int vis_pmsg_head = 0;
static float vis_inter_flash[2] = {0,0};
static float screen_shake = 0.0f;
static int active_tab = 0;

/*   HELPER FUNCTIONS
*/
static void os_label_add(float x, float y, const char *concept, const char *label, Color col) {
    pthread_mutex_lock(&vis_mutex);
    int i = os_label_idx % MAX_OS_LABELS;
    strncpy(os_labels[i].label,   label,   47);
    strncpy(os_labels[i].concept, concept, 31);
    os_labels[i].x      = x;
    os_labels[i].y      = y;
    os_labels[i].alpha  = 1.0f;
    os_labels[i].life   = 4.0f;
    os_labels[i].col    = col;
    os_labels[i].active = 1;
    os_label_idx++;
    pthread_mutex_unlock(&vis_mutex);
}

static void vis_log_add(const char *msg, Color col) {
    pthread_mutex_lock(&vis_mutex);
    int idx = (vis_log_head + vis_log_count) % MAX_LOG_ENTRIES;
    strncpy(vis_log[idx].msg, msg, LOG_MSG_LEN-1);
    vis_log[idx].col = col;
    if (vis_log_count < MAX_LOG_ENTRIES) vis_log_count++;
    else vis_log_head = (vis_log_head + 1) % MAX_LOG_ENTRIES;
    pthread_mutex_unlock(&vis_mutex);
}

static void vis_pmsg_add(const char *txt, int dir) {
    pthread_mutex_lock(&vis_mutex);
    int i = vis_pmsg_head % MAX_PMSG;
    strncpy(vis_pmsgs[i].txt, txt, 79);
    vis_pmsgs[i].progress = 0;
    vis_pmsgs[i].dir = dir;
    vis_pmsgs[i].alive = 1;
    vis_pmsgs[i].alpha = 1.0f;
    vis_pmsg_head++;
    pthread_mutex_unlock(&vis_mutex);
}

static VisVehicle *vis_get_veh(int id) {
    for (int i = 0; i < vis_veh_count; i++)
        if (vis_vehicles[i].id == id) return &vis_vehicles[i];
    if (vis_veh_count < MAX_VIS_VEHICLES) {
        VisVehicle *v = &vis_vehicles[vis_veh_count++];
        memset(v, 0, sizeof(*v));
        v->id = id;
        v->active = 1;
        return v;
    }
    return NULL;
}

/* Queue up to 8 road-corner waypoints. tx/ty advances to each in sequence.
 * Call with vis_mutex held. n must be 1..8.                               */
static void vis_set_waypoints(VisVehicle *vv, float *xs, float *ys, int n) {
    if (!vv || n <= 0 || n > 8) return;
    for (int i = 0; i < n; i++) { vv->wp_x[i] = xs[i]; vv->wp_y[i] = ys[i]; }
    vv->wp_count = n;
    vv->wp_idx   = 0;
    vv->tx = xs[0];
    vv->ty = ys[0];
}

static const char *type_str(int t) {
    switch(t) {
        case TYPE_AMBULANCE: return "Ambulance";
        case TYPE_FIRETRUCK: return "Firetruck";
        case TYPE_BUS:       return "Bus";
        case TYPE_CAR:       return "Car";
        case TYPE_BIKE:      return "Bike";
        case TYPE_TRACTOR:   return "Tractor";
        default:             return "Unknown";
    }
}

static const char *type_short(int t) {
    switch(t) {
        case TYPE_AMBULANCE: return "AMB";
        case TYPE_FIRETRUCK: return "FTK";
        case TYPE_BUS:       return "BUS";
        case TYPE_CAR:       return "CAR";
        case TYPE_BIKE:      return "BIK";
        case TYPE_TRACTOR:   return "TRC";
        default:             return "???";
    }
}

static const char *inter_str(int i) {
    return (i == INTERSECTION_F10) ? "F10" : "F11";
}

static Color veh_color(int type) {
    switch(type) {
        case TYPE_AMBULANCE: return (Color){240, 60,  60,  255};
        case TYPE_FIRETRUCK: return (Color){255, 100, 20,  255};
        case TYPE_BUS:       return (Color){255, 210, 30,  255};
        case TYPE_CAR:       return (Color){180, 210, 250, 255};
        case TYPE_BIKE:      return (Color){60,  230, 90,  255};
        case TYPE_TRACTOR:   return (Color){160, 120, 50,  255};
        default:             return WHITE;
    }
}

static Color prio_color(int p) {
    if (p == PRIORITY_EMERGENCY) return (Color){255, 60, 60, 255};
    if (p == PRIORITY_BUS)       return (Color){255, 200, 0, 255};
    return (Color){80, 200, 80, 255};
}
/*   PARKING LOGIC
   Per the project spec, the primary Parking Lot is attached to the
   F10 intersection.  All vehicles first attempt the F10 lot.
   Only if the F10 queue is full do they fall back to the F11
   overflow lot  */
static int try_park(Vehicle *v) {
    /*
     * PRIMARY lot  = F10 (spec: "Parking Lot attached to F10")
     * OVERFLOW lot = F11 (activated only when F10 queue is full)
     * Emergency vehicles never reach this function (checked in vehicle_thread).
     */

    /*  Try F10 primary lot first, regardless of vehicle's intersection  */
    int used_inter = INTERSECTION_F10;   /* assume primary */

    if (sem_trywait(&sem_queue_f10) != 0) {
        /* F10 queue full ,try F11 overflow */
        printf(YLW_TXT "[PARKING-F10] V%d %s: F10 queue full, trying F11 overflow\n"
               RST_TXT, v->id, type_short(v->type));
        char lmsg0[LOG_MSG_LEN];
        snprintf(lmsg0, sizeof(lmsg0), "V%d %s: F10 full → F11 overflow", v->id, type_short(v->type));
        vis_log_add(lmsg0, YELLOW);

        if (sem_trywait(&sem_queue_f11) != 0) {
            /* Both lots full — vehicle will cross instead */
            printf(YLW_TXT "[PARKING] V%d %s: both lots full, will cross\n"
                   RST_TXT, v->id, type_short(v->type));
            char lmsg1[LOG_MSG_LEN];
            snprintf(lmsg1, sizeof(lmsg1), "V%d %s: both lots full → crossing", v->id, type_short(v->type));
            vis_log_add(lmsg1, YELLOW);
            return 0;
        }
        used_inter = INTERSECTION_F11;   /* using overflow */
    }

    /* ── Step 2: Got a queue slot — update visuals ── */
    sem_t *sem_spots = (used_inter == INTERSECTION_F10) ? &sem_spots_f10 : &sem_spots_f11;
    int   *slots     = (used_inter == INTERSECTION_F10) ? vis_park_slots_f10 : vis_park_slots_f11;
    const char *loc  = inter_str(used_inter);

    pthread_mutex_lock(&vis_mutex);
    if (used_inter == INTERSECTION_F10) vis_parkq_f10++;
    else                                vis_parkq_f11++;
    VisVehicle *vv = vis_get_veh(v->id);
    if (vv) vv->state = 2;
    pthread_mutex_unlock(&vis_mutex);

    /*  sem_wait for a parking spot */
    float ox = (used_inter == INTERSECTION_F10) ? PK_F10_X + 80 : PK_F11_X + 80;
    os_label_add(ox, PK_F10_Y - 30, "sem_wait()", "Semaphore: waiting for spot", (Color){80,160,255,255});

    sem_wait(sem_spots);   /* block until a spot is free — NEVER blocks the intersection */

    /*  Claim a physical slot  */
    int slot = -1;
    pthread_mutex_lock(&vis_mutex);
    for (int i = 0; i < PARKING_SPOTS; i++) {
        if (!slots[i]) { slots[i] = 1; slot = i; break; }
    }
    if (used_inter == INTERSECTION_F10) vis_park_f10++;
    else                                vis_park_f11++;
    pthread_mutex_unlock(&vis_mutex);

    v->parked_slot  = slot;
    v->intersection = used_inter;   /* update so stats go to correct lot */
    printf(GRN_TXT "[PARKING-%s] Vehicle %2d %s: PARKED in slot %d!\n"
           RST_TXT, loc, v->id, type_short(v->type), slot);

    char lmsg[LOG_MSG_LEN];
    snprintf(lmsg, sizeof(lmsg), "V%d %s parked @%s slot %d", v->id, type_short(v->type), loc, slot);
    vis_log_add(lmsg, GREEN);

    /*  Stay parked — drive to lot via road network 
     * Route: current position → horizontal road (ROAD_Y) → lot column → lot slot
     * This keeps the vehicle ON roads the entire journey.              */
    int park_time = 3 + rand() % 3;
    float pk_lot_x = (used_inter == INTERSECTION_F10) ? (float)PK_F10_X : (float)PK_F11_X;
    float pk_lot_y = (used_inter == INTERSECTION_F10) ? (float)PK_F10_Y : (float)PK_F11_Y;
    float slot_x = pk_lot_x + (slot % 5) * 34 + 16;
    float slot_y = pk_lot_y + (slot / 5) * 30 + 14;

    pthread_mutex_lock(&vis_mutex);
    VisVehicle *pv = vis_get_veh(v->id);
    if (pv) {
        float cur_x = pv->x, cur_y = pv->y;
        /*
         * Road route to parking slot (horizontal-first, same y level):
         *   wp0 = slide horizontally to lot column (staying at current y)
         *   wp1 = turn south into lot entrance
         *   wp2 = final slot position
         */
        (void)cur_x;   /* x already encoded in slot_x */
        float wxs[3] = { slot_x,  slot_x,   slot_x };
        float wys[3] = { cur_y,   pk_lot_y, slot_y };
        vis_set_waypoints(pv, wxs, wys, 3);
        pv->progress = 0;
    }
    pthread_mutex_unlock(&vis_mutex);

    /* Wait for driving animation to reach slot */
    for (int tt = 0; tt < park_time * 10 && running; tt++) {
        pthread_mutex_lock(&vis_mutex);
        VisVehicle *ppv = vis_get_veh(v->id);
        if (ppv) ppv->progress = (float)tt / (park_time * 10.0f);
        pthread_mutex_unlock(&vis_mutex);
        usleep(150000);
    }

    sleep(4);

    /*  Leave and release semaphores */
    pthread_mutex_lock(&vis_mutex);
    if (slot >= 0) slots[slot] = 0;
    if (used_inter == INTERSECTION_F10) {
        vis_park_f10--;
        vis_parkq_f10--;
        vis_stat_parked[0]++;
    } else {
        vis_park_f11--;
        vis_parkq_f11--;
        vis_stat_parked[1]++;
    }
    VisVehicle *lv = vis_get_veh(v->id);
    if (lv) { lv->state = 0; lv->progress = 0; }
    pthread_mutex_unlock(&vis_mutex);

    /*  sem_post — releases spot and queue slot */
    os_label_add(ox, PK_F10_Y - 30, "sem_post()", "Semaphore: releasing spot", (Color){60,230,80,255});

    sem_post(sem_spots);
    if (used_inter == INTERSECTION_F10) sem_post(&sem_queue_f10);
    else                                sem_post(&sem_queue_f11);

    printf(BLU_TXT "[PARKING-%s] Vehicle %2d %s: left after %ds\n"
           RST_TXT, loc, v->id, type_short(v->type), park_time);
    return 1;
}

/*
   TRAFFIC LIGHT CONTROL */
static void set_light(int intersection, int green_flag) {
    if (intersection == INTERSECTION_F10) {
        pthread_mutex_lock(&mutex_f10);
        light_f10_green = green_flag;
        if (green_flag) {
            /* Wake buses first (priority), then all normal vehicles */
            pthread_cond_broadcast(&cond_f10_bus);
            pthread_cond_broadcast(&cond_f10_green);
        }
        pthread_mutex_unlock(&mutex_f10);
    } else {
        pthread_mutex_lock(&mutex_f11);
        light_f11_green = green_flag;
        if (green_flag) {
            pthread_cond_broadcast(&cond_f11_bus);
            pthread_cond_broadcast(&cond_f11_green);
        }
        pthread_mutex_unlock(&mutex_f11);
    }
}

static void wait_for_green(Vehicle *v) {
    struct timespec start, end;
    clock_gettime(CLOCK_MONOTONIC, &start);

    /* OS CONCEPT: mutex + cond var */
    float ox = (v->intersection == INTERSECTION_F10) ? F10_CX : F11_CX;
    os_label_add(ox, F10_CY - 80, "pthread_cond_wait()", "Mutex+CondVar: waiting for GREEN", (Color){255,200,50,255});

    /*
     * Non-conflicting movement: approach axis
     *   North/South (approach 0,1) → axis bit 0 (NS)
     *   East/West   (approach 2,3) → axis bit 1 (EW)
     * Two vehicles on the SAME axis may cross together.
     * Two vehicles on DIFFERENT axes conflict and must wait for each other.
     */
    int approach = 0;
    if      (strncmp(v->origin, "North", 5) == 0) approach = 0;
    else if (strncmp(v->origin, "South", 5) == 0) approach = 1;
    else if (strncmp(v->origin, "East",  4) == 0) approach = 2;
    else                                            approach = 3;
    int my_axis = (approach <= 1) ? 1 : 2;   /* bit0=NS=1, bit1=EW=2 */
    int opp_axis = (my_axis == 1) ? 2 : 1;

    if (v->intersection == INTERSECTION_F10) {
        pthread_mutex_lock(&mutex_f10);

        if (v->priority == PRIORITY_BUS) {
            /* BUS: waits on dedicated cond — woken before NORMAL vehicles */
            while (!light_f10_green || emergency_active ||
                   (f10_crossing_dir != 0 && f10_crossing_dir != my_axis)) {
                printf(YLW_TXT "[WAIT-F10-BUS] V%d BUS waiting (priority)...\n" RST_TXT, v->id);
                pthread_cond_wait(&cond_f10_bus, &mutex_f10);
            }
        } else {
            /* NORMAL: waits on standard cond, also yields if conflicting axis active */
            while (!light_f10_green || emergency_active ||
                   (f10_crossing_dir != 0 && f10_crossing_dir != my_axis)) {
                printf(YLW_TXT "[WAIT-F10] V%d %s waiting at red...\n" RST_TXT, v->id, type_short(v->type));
                pthread_cond_wait(&cond_f10_green, &mutex_f10);
            }
        }

        f10_crossing_count++;
        f10_crossing_dir |= my_axis;   /* mark this axis as active */
        (void)opp_axis;
        pthread_mutex_unlock(&mutex_f10);

    } else {
        pthread_mutex_lock(&mutex_f11);

        if (v->priority == PRIORITY_BUS) {
            while (!light_f11_green || emergency_active ||
                   (f11_crossing_dir != 0 && f11_crossing_dir != my_axis)) {
                printf(YLW_TXT "[WAIT-F11-BUS] V%d BUS waiting (priority)...\n" RST_TXT, v->id);
                pthread_cond_wait(&cond_f11_bus, &mutex_f11);
            }
        } else {
            while (!light_f11_green || emergency_active ||
                   (f11_crossing_dir != 0 && f11_crossing_dir != my_axis)) {
                printf(YLW_TXT "[WAIT-F11] V%d %s waiting at red...\n" RST_TXT, v->id, type_short(v->type));
                pthread_cond_wait(&cond_f11_green, &mutex_f11);
            }
        }

        f11_crossing_count++;
        f11_crossing_dir |= my_axis;
        pthread_mutex_unlock(&mutex_f11);
    }

    clock_gettime(CLOCK_MONOTONIC, &end);
    v->wait_time = (end.tv_sec - start.tv_sec) + (end.tv_nsec - start.tv_nsec) / 1e9;
    printf(GRN_TXT "[CROSS] V%d %s got GREEN after %.1fs wait\n" RST_TXT,
           v->id, type_short(v->type), v->wait_time);
}

static void done_crossing(Vehicle *v) {
    int approach = 0;
    if      (strncmp(v->origin, "North", 5) == 0) approach = 0;
    else if (strncmp(v->origin, "South", 5) == 0) approach = 1;
    else if (strncmp(v->origin, "East",  4) == 0) approach = 2;
    else                                            approach = 3;
    int my_axis = (approach <= 1) ? 1 : 2;

    if (v->intersection == INTERSECTION_F10) {
        pthread_mutex_lock(&mutex_f10);
        f10_crossing_count--;
        /* Clear axis bit only when no more vehicles from this axis are crossing */
        if (f10_crossing_count == 0)
            f10_crossing_dir = 0;
        /* Wake all waiters — opposite-axis vehicles can now check again */
        pthread_cond_broadcast(&cond_f10_bus);
        pthread_cond_broadcast(&cond_f10_green);
        pthread_mutex_unlock(&mutex_f10);
    } else {
        pthread_mutex_lock(&mutex_f11);
        f11_crossing_count--;
        if (f11_crossing_count == 0)
            f11_crossing_dir = 0;
        pthread_cond_broadcast(&cond_f11_bus);
        pthread_cond_broadcast(&cond_f11_green);
        pthread_mutex_unlock(&mutex_f11);
    }
    (void)my_axis;
}

/* 
   EMERGENCY HANDLER
*/
static void handle_emergency(Vehicle *v) {
    pthread_mutex_lock(&mutex_emergency);
    emergency_active = 1;
    pthread_mutex_unlock(&mutex_emergency);

    set_light(INTERSECTION_F10, 0);
    set_light(INTERSECTION_F11, 0);

    pthread_mutex_lock(&vis_mutex);
    vis_light_f10 = 0;
    vis_light_f11 = 0;
    vis_emergency = 1;
    vis_inter_flash[v->intersection] = 5.0f;
    screen_shake = 12.0f;
    VisVehicle *ev = vis_get_veh(v->id);
    if (ev) ev->flash = 8.0f;
    if (v->intersection == INTERSECTION_F10) vis_stat_emg[0]++;
    else vis_stat_emg[1]++;
    pthread_mutex_unlock(&vis_mutex);

    /* signal / IPC pipe */
    float ox = (v->intersection == INTERSECTION_F10) ? F10_CX : F11_CX;
    os_label_add(ox, F10_CY + 60, "write(pipe_fd)", "IPC: Emergency via pipe!", (Color){255,80,80,255});

    printf(RED_TXT "\n[EMERGENCY] %s at %s!\n" RST_TXT, type_str(v->type), inter_str(v->intersection));

    const char *emg_msg = MSG_EMERGENCY_COMING;
    char pmsg[64];
    /* Notify BOTH controllers via both pipes — spec requires both intersections to clear */
    write(pipe_f10_to_f11[1], emg_msg, strlen(emg_msg)+1);
    write(pipe_f11_to_f10[1], emg_msg, strlen(emg_msg)+1);
    if (v->intersection == INTERSECTION_F10) {
        snprintf(pmsg, sizeof(pmsg), "EMG:F10->F11");
        vis_pmsg_add(pmsg, 0);
    } else {
        snprintf(pmsg, sizeof(pmsg), "EMG:F11->F10");
        vis_pmsg_add(pmsg, 1);
    }
    /* Also animate the return pipe direction for visual clarity */
    if (v->intersection == INTERSECTION_F10) {
        snprintf(pmsg, sizeof(pmsg), "EMG:F11 ACK");
        vis_pmsg_add(pmsg, 1);
    } else {
        snprintf(pmsg, sizeof(pmsg), "EMG:F10 ACK");
        vis_pmsg_add(pmsg, 0);
    }

    sleep(3);

    /*
     * Emergency vehicle moves in ONE direction only — straight through
     * both intersections and off the far screen edge. No U-turn, no return.
     *
     * Exit side: East-origin vehicles came from the right → exit left.
     *            All others → exit right.
     */
    int lane_y = ROAD_Y - ROAD_LANE_INBOUND;

    int going_right = (strncmp(v->origin, "East", 4) == 0) ? 0 : 1;
    float exit_x = going_right ? (float)(SIM_AREA_W + SPAWN_MARGIN)
                                : (float)(-SPAWN_MARGIN);

    pthread_mutex_lock(&vis_mutex);
    VisVehicle *ec = vis_get_veh(v->id);
    if (ec) {
        /* Single segment: wherever the vehicle is now → off-screen far edge.
         * Keep the vehicle's current y so it travels in a pure straight line
         * with no vertical snap / U-shape artefact.                          */
        float wxs[2] = { ec->x,   exit_x   };
        float wys[2] = { ec->y,   ec->y    };
        vis_set_waypoints(ec, wxs, wys, 2);
        ec->flash = 8.0f;
    }
    pthread_mutex_unlock(&vis_mutex);

    for (int t = 0; t < 45 && running; t++) {
        pthread_mutex_lock(&vis_mutex);
        VisVehicle *cv2 = vis_get_veh(v->id);
        if (cv2) cv2->progress = (float)t / 45.0f;
        pthread_mutex_unlock(&vis_mutex);
        usleep(100000);
    }

    sleep(2);

    pthread_mutex_lock(&mutex_emergency);
    emergency_active = 0;
    pthread_mutex_unlock(&mutex_emergency);

    set_light(INTERSECTION_F10, 1);
    set_light(INTERSECTION_F11, 1);

    pthread_mutex_lock(&vis_mutex);
    vis_light_f10 = 1;
    vis_light_f11 = 1;
    vis_emergency = 0;
    pthread_mutex_unlock(&vis_mutex);

    os_label_add(ox, F10_CY + 60, "SIGINT handler", "Signal: lights restored", (Color){60,230,80,255});
    vis_log_add("Emergency cleared — intersections restored", GREEN);
}

/* 
   VEHICLE FACTORY
*/
static Vehicle *create_vehicle(int id) {
    static const char *origins[] = {"North", "South", "East", "West"};
    static const char *dirs[]    = {"Straight", "Left", "Right"};

    Vehicle *v = malloc(sizeof(Vehicle));
    if (!v) { perror("malloc"); exit(1); }

    v->id = id;
    v->arrival_time = time(NULL);
    v->intersection = rand() % 2;
    v->direction    = rand() % 3;
    v->type         = rand() % 6;
    v->will_park    = 0;
    v->parked_slot  = -1;
    v->wait_time    = 0;

    strncpy(v->type_name, type_str(v->type), sizeof(v->type_name)-1);
    strncpy(v->origin,    origins[rand()%4],   sizeof(v->origin)-1);
    strncpy(v->destination, dirs[v->direction], sizeof(v->destination)-1);

    if (v->type == TYPE_AMBULANCE || v->type == TYPE_FIRETRUCK)
        v->priority = PRIORITY_EMERGENCY;
    else if (v->type == TYPE_BUS)
        v->priority = PRIORITY_BUS;
    else
        v->priority = PRIORITY_NORMAL;

    if (v->priority != PRIORITY_EMERGENCY &&
        (v->type == TYPE_CAR || v->type == TYPE_BIKE ||
         v->type == TYPE_TRACTOR || v->type == TYPE_BUS)) {
        v->will_park = (rand() % 10) < 6;
    }

    return v;
}

/* 
   VEHICLE THREAD
 */
static void *vehicle_thread(void *arg) {
    Vehicle *v = (Vehicle *)arg;

    printf(CYN_TXT "\n[THREAD V%d] %s | Priority: %s | %s -> %s | Intersection: %s\n" RST_TXT,
           v->id, v->type_name,
           v->priority == PRIORITY_EMERGENCY ? "EMERGENCY" : (v->priority == PRIORITY_BUS ? "BUS" : "NORMAL"),
           v->origin, v->destination, inter_str(v->intersection));

    float ox = (v->intersection == INTERSECTION_F10) ? F10_CX - 60 + v->id*4 : F11_CX - 60 + v->id*4;
    os_label_add(ox, 60 + (v->id % 4)*14, "pthread_create()", "Thread spawned for vehicle", (Color){100,200,255,255});

    int cx = (v->intersection == INTERSECTION_F10) ? F10_CX : F11_CX;
    int cy = ROAD_Y;

    /* Approach direction */
    int approach = 0;
    if      (strncmp(v->origin, "North", 5) == 0) approach = 0;
    else if (strncmp(v->origin, "South", 5) == 0) approach = 1;
    else if (strncmp(v->origin, "East",  4) == 0) approach = 2;
    else                                            approach = 3;

    /*
     * Right-hand traffic lane offsets:
     *   North (entering from top, travelling south):
     *       inbound lane  x = cx + ROAD_LANE_INBOUND
     *   South (entering from bottom, travelling north):
     *       inbound lane  x = cx - ROAD_LANE_INBOUND
     *   East (entering from right, travelling west):
     *       inbound lane  y = cy + ROAD_LANE_INBOUND
     *   West (entering from left, travelling east):
     *       inbound lane  y = cy - ROAD_LANE_INBOUND
     */

    /* Compute all road waypoints for this vehicle's journey  */

    /*  Spawn point : off screen edge, on road */
    float spx, spy;
    switch (approach) {
        case 0:  spx = cx + ROAD_LANE_INBOUND;   spy = -SPAWN_MARGIN;        break;
        case 1:  spx = cx - ROAD_LANE_INBOUND;   spy = SCREEN_H+SPAWN_MARGIN;break;
        case 2:  spx = SIM_AREA_W+SPAWN_MARGIN;  spy = cy + ROAD_LANE_INBOUND; break;
        default: spx = -SPAWN_MARGIN;             spy = cy - ROAD_LANE_INBOUND; break;
    }

    /*  Stop line  just before intersection box */
    float stx, sty;
    switch (approach) {
        case 0:  stx = cx + ROAD_LANE_INBOUND;   sty = cy - ROAD_HW - 4;    break;
        case 1:  stx = cx - ROAD_LANE_INBOUND;   sty = cy + ROAD_HW + 4;    break;
        case 2:  stx = cx + ROAD_HW + 4;         sty = cy + ROAD_LANE_INBOUND; break;
        default: stx = cx - ROAD_HW - 4;         sty = cy - ROAD_LANE_INBOUND; break;
    }

    /* 3. Intersection centre */
    float icx = cx, icy = cy;

    /* 4. Exit waypoint — vehicle leaves intersection onto outbound road arm */
    /*    Direction depends on v->direction (Straight/Left/Right)           */
    float ex1, ey1;   /* first point after intersection */
    float ex2, ey2;   /* road edge / screen exit point  */

    if (v->direction == DIR_STRAIGHT) {
        /* Straight: continue on same axis, opposite side */
        switch (approach) {
            case 0:  ex1 = cx + ROAD_LANE_INBOUND; ey1 = cy + ROAD_HW + 4;
                     ex2 = cx + ROAD_LANE_INBOUND; ey2 = SCREEN_H + SPAWN_MARGIN; break;
            case 1:  ex1 = cx - ROAD_LANE_INBOUND; ey1 = cy - ROAD_HW - 4;
                     ex2 = cx - ROAD_LANE_INBOUND; ey2 = -SPAWN_MARGIN;           break;
            case 2:  ex1 = cx - ROAD_HW - 4;       ey1 = cy + ROAD_LANE_INBOUND;
                     ex2 = -SPAWN_MARGIN;           ey2 = cy + ROAD_LANE_INBOUND; break;
            default: ex1 = cx + ROAD_HW + 4;       ey1 = cy - ROAD_LANE_INBOUND;
                     ex2 = SIM_AREA_W+SPAWN_MARGIN; ey2 = cy - ROAD_LANE_INBOUND; break;
        }
    } else if (v->direction == DIR_LEFT) {
        /* Turn left: 90° counter-clockwise */
        switch (approach) {
            case 0:  ex1 = cx - ROAD_LANE_INBOUND; ey1 = cy;
                     ex2 = -SPAWN_MARGIN;           ey2 = cy - ROAD_LANE_INBOUND; break;
            case 1:  ex1 = cx + ROAD_LANE_INBOUND; ey1 = cy;
                     ex2 = SIM_AREA_W+SPAWN_MARGIN; ey2 = cy + ROAD_LANE_INBOUND; break;
            case 2:  ex1 = cx;                      ey1 = cy - ROAD_LANE_INBOUND;
                     ex2 = cx + ROAD_LANE_INBOUND;  ey2 = -SPAWN_MARGIN;          break;
            default: ex1 = cx;                      ey1 = cy + ROAD_LANE_INBOUND;
                     ex2 = cx - ROAD_LANE_INBOUND;  ey2 = SCREEN_H+SPAWN_MARGIN;  break;
        }
    } else {
        /* Turn right: 90° clockwise */
        switch (approach) {
            case 0:  ex1 = cx + ROAD_LANE_INBOUND; ey1 = cy;
                     ex2 = SIM_AREA_W+SPAWN_MARGIN; ey2 = cy - ROAD_LANE_INBOUND; break;
            case 1:  ex1 = cx - ROAD_LANE_INBOUND; ey1 = cy;
                     ex2 = -SPAWN_MARGIN;           ey2 = cy + ROAD_LANE_INBOUND; break;
            case 2:  ex1 = cx;                      ey1 = cy + ROAD_LANE_INBOUND;
                     ex2 = cx - ROAD_LANE_INBOUND;  ey2 = SCREEN_H+SPAWN_MARGIN;  break;
            default: ex1 = cx;                      ey1 = cy - ROAD_LANE_INBOUND;
                     ex2 = cx + ROAD_LANE_INBOUND;  ey2 = -SPAWN_MARGIN;          break;
        }
    }

    /* Initialise visual vehicle at spawn point */
    pthread_mutex_lock(&vis_mutex);
    VisVehicle *vv = vis_get_veh(v->id);
    if (vv) {
        vv->type         = v->type;
        vv->priority     = v->priority;
        vv->intersection = v->intersection;
        vv->state        = 0;
        vv->active       = 1;
        strncpy(vv->type_name, type_short(v->type), 7);
        strncpy(vv->origin,    v->origin, 7);
        strncpy(vv->destination, v->destination, 7);
        vv->x  = spx;   vv->y  = spy;
        vv->tx = spx;   vv->ty = spy;
        vv->wp_count = 0;  vv->wp_idx = 0;
        vv->arrival_time = v->arrival_time;
        vv->will_park    = v->will_park;

        /* Waypoints: spawn → stop line (vehicle drives in on road) */
        float wxs[2] = { spx, stx };
        float wys[2] = { spy, sty };
        vis_set_waypoints(vv, wxs, wys, 2);
    }
    pthread_mutex_unlock(&vis_mutex);

    char lmsg[LOG_MSG_LEN];
    snprintf(lmsg, sizeof(lmsg), "V%d %s spawned @%s [%s]",
             v->id, type_short(v->type), inter_str(v->intersection), v->origin);
    vis_log_add(lmsg, SKYBLUE);

    int delay = (rand() % (SPAWN_DELAY_MAX - SPAWN_DELAY_MIN) + SPAWN_DELAY_MIN) * 1000;
    usleep(delay);

    /* ── Point vehicle at stop line (already queued, just make sure)  */
    pthread_mutex_lock(&vis_mutex);
    VisVehicle *mv = vis_get_veh(v->id);
    if (mv) { mv->tx = stx; mv->ty = sty; }
    pthread_mutex_unlock(&vis_mutex);

    /* ── EMERGENCY VEHICLES bypass normal flow ── */
    if (v->priority == PRIORITY_EMERGENCY) {
        handle_emergency(v);

        pthread_mutex_lock(&vis_mutex);
        VisVehicle *dv = vis_get_veh(v->id);
        if (dv) { dv->state = 3; dv->active = 0; }
        vis_stat_done++;
        pthread_mutex_unlock(&vis_mutex);

        snprintf(lmsg, sizeof(lmsg), "V%d %s DONE (emergency)", v->id, type_short(v->type));
        vis_log_add(lmsg, RED);
        free(v);
        return NULL;
    }

    /* ── PARKING PHASE ──────────────────────────────────────────────
     * try_park() is called BEFORE wait_for_green() so that a vehicle
     * waiting for a parking spot NEVER blocks the intersection.
     * Semaphore accounting:
     *   sem_trywait(sem_queue) — non-blocking queue reservation
     *   sem_wait(sem_spots)    — blocking wait for a free spot
     *                           (happens AWAY from the intersection)
     * Exactly ONE of the three paths below increments vis_stat_done:
     *   Path A — emergency handled  (above)         → done++, return
     *   Path B — successfully parked (try_park==1)  → done++, return
     *   Path C — parking skipped/full, crosses      → done++ at end
     * These paths are mutually exclusive — no double-count possible.
     * ───────────────────────────────────────────────────────────── */
    if (v->will_park) {
        int parked = try_park(v);
        if (parked) {
            pthread_mutex_lock(&vis_mutex);
            VisVehicle *dv = vis_get_veh(v->id);
            if (dv) { dv->state = 3; dv->active = 0; }
            vis_stat_done++;
            pthread_mutex_unlock(&vis_mutex);

            snprintf(lmsg, sizeof(lmsg), "V%d %s DONE (parked)", v->id, type_short(v->type));
            vis_log_add(lmsg, GREEN);
            free(v);
            return NULL;
        }
    }

    /* ── WAIT FOR GREEN ── */
    wait_for_green(v);

    /* ── CROSSING — road-only waypoints: stop→centre→exit ── */
    pthread_mutex_lock(&vis_mutex);
    VisVehicle *cv = vis_get_veh(v->id);
    if (cv) {
        cv->state    = 1;
        cv->progress = 0;
        /* Queue: stop line → intersection centre → exit arm → off screen */
        float wxs[4] = { stx,  icx,  ex1,  ex2  };
        float wys[4] = { sty,  icy,  ey1,  ey2  };
        vis_set_waypoints(cv, wxs, wys, 4);
    }
    vis_inter_flash[v->intersection] = 2.0f;
    pthread_mutex_unlock(&vis_mutex);

    printf(GRN_TXT "[CROSSING] V%d %s crossing %s\n" RST_TXT,
           v->id, type_short(v->type), inter_str(v->intersection));

    /* Wait long enough for the visual to traverse all 4 waypoints */
    int cross_time  = (v->type == TYPE_BUS || v->type == TYPE_TRACTOR) ? CROSSING_TIME_SLOW : CROSSING_TIME_FAST;
    int cross_steps = cross_time * 22;
    for (int tt = 0; tt < cross_steps && running; tt++) {
        pthread_mutex_lock(&vis_mutex);
        VisVehicle *xv = vis_get_veh(v->id);
        if (xv) xv->progress = (float)tt / (float)cross_steps;
        pthread_mutex_unlock(&vis_mutex);
        usleep(100000);
    }

    done_crossing(v);

    pthread_mutex_lock(&vis_mutex);
    VisVehicle *xv = vis_get_veh(v->id);
    if (xv) {
        if (v->intersection == INTERSECTION_F10) vis_stat_crossed[0]++;
        else vis_stat_crossed[1]++;
        xv->state  = 3;
        xv->active = 0;
    }
    vis_stat_done++;   /* PATH C */
    pthread_mutex_unlock(&vis_mutex);

    printf(MAG_TXT "[DONE] Vehicle %2d %s completed crossing (waited %.1fs)\n\n" RST_TXT,
           v->id, type_short(v->type), v->wait_time);
    snprintf(lmsg, sizeof(lmsg), "V%d %s crossed %s (%.1fs wait)",
             v->id, type_short(v->type), inter_str(v->intersection), v->wait_time);
    vis_log_add(lmsg, MAGENTA);

    free(v);
    return NULL;
}

/*  CONTROLLER PROCESS */
static void wait_intersection_clear(int intersection) {
    if (intersection == INTERSECTION_F10) {
        pthread_mutex_lock(&mutex_f10);
        while (f10_crossing_count > 0)
            pthread_cond_wait(&cond_f10_green, &mutex_f10);
        pthread_mutex_unlock(&mutex_f10);
    } else {
        pthread_mutex_lock(&mutex_f11);
        while (f11_crossing_count > 0)
            pthread_cond_wait(&cond_f11_green, &mutex_f11);
        pthread_mutex_unlock(&mutex_f11);
    }
}

static void controller_loop(int my_intersection, int read_fd, int write_fd) {
    const char *name = inter_str(my_intersection);
    char buf[64];
    int cycle_count = 0;

    int flags = fcntl(read_fd, F_GETFL, 0);
    fcntl(read_fd, F_SETFL, flags | O_NONBLOCK);

    printf(GRN_TXT "[CTRL-%s] Controller started (PID=%d)\n" RST_TXT, name, getpid());

    while (running) {
        set_light(my_intersection, 1);
        printf("[CTRL-%s] GREEN phase (cycle %d)\n", name, ++cycle_count);

        for (int t = 0; t < GREEN_DURATION && running; t++) {
            sleep(1);
            ssize_t n = read(read_fd, buf, sizeof(buf)-1);
            if (n > 0) {
                buf[n] = '\0';
                if (strncmp(buf, MSG_EMERGENCY_COMING, strlen(MSG_EMERGENCY_COMING)) == 0) {
                    set_light(my_intersection, 0);
                    wait_intersection_clear(my_intersection);
                    while (emergency_active && running) sleep(1);
                    break;
                }
                if (strncmp(buf, MSG_SHUTDOWN, strlen(MSG_SHUTDOWN)) == 0) return;
            }
        }
        if (!running) break;

        set_light(my_intersection, 0);
        printf("[CTRL-%s] RED phase\n", name);
        sleep(RED_DURATURE);
    }
}

static void run_controller_f10(void) {
    close(pipe_f10_to_f11[0]);
    close(pipe_f11_to_f10[1]);
    controller_loop(INTERSECTION_F10, pipe_f11_to_f10[0], pipe_f10_to_f11[1]);
    exit(0);
}

static void run_controller_f11(void) {
    close(pipe_f11_to_f10[0]);
    close(pipe_f10_to_f11[1]);
    controller_loop(INTERSECTION_F11, pipe_f10_to_f11[0], pipe_f11_to_f10[1]);
    exit(0);
}

/*  VISUAL UPDATE*/
static void update_vis(float dt) {
    pthread_mutex_lock(&vis_mutex);

    for (int i = 0; i < vis_veh_count; i++) {
        VisVehicle *v = &vis_vehicles[i];
        if (!v->active) continue;
        if (v->flash > 0) { v->flash -= dt * 2.5f; if (v->flash < 0) v->flash = 0; }

        float dx = v->tx - v->x, dy = v->ty - v->y;
        float dist = sqrtf(dx*dx + dy*dy);
        if (dist > 2.0f) {
            /* Angle faces direction of travel — road-aligned */
            v->angle = atan2f(dy, dx) * 180.0f / 3.14159f;
            float spd = fminf(130.0f * dt, dist);
            v->x += dx / dist * spd;
            v->y += dy / dist * spd;
        } else {
            v->x = v->tx;
            v->y = v->ty;
            /* Advance to next waypoint if one is queued */
            if (v->wp_count > 0 && v->wp_idx + 1 < v->wp_count) {
                v->wp_idx++;
                v->tx = v->wp_x[v->wp_idx];
                v->ty = v->wp_y[v->wp_idx];
            }
        }
    }

    for (int i = 0; i < MAX_PMSG; i++) {
        if (!vis_pmsgs[i].alive) continue;
        vis_pmsgs[i].progress += dt * 0.5f;
        vis_pmsgs[i].alpha    = 1.0f - vis_pmsgs[i].progress;
        if (vis_pmsgs[i].progress >= 1.0f) vis_pmsgs[i].alive = 0;
    }

    for (int i = 0; i < 2; i++) {
        if (vis_inter_flash[i] > 0) {
            vis_inter_flash[i] -= dt * 3.0f;
            if (vis_inter_flash[i] < 0) vis_inter_flash[i] = 0;
        }
    }
    if (screen_shake > 0) { screen_shake -= dt * 15.0f; if (screen_shake < 0) screen_shake = 0; }

    for (int i = 0; i < MAX_OS_LABELS; i++) {
        if (!os_labels[i].active) continue;
        os_labels[i].life -= dt;
        /* Fade in for first 0.4s, then hold, then fade out */
        float life_frac = os_labels[i].life / 4.0f;
        if (life_frac > 0.9f)
            os_labels[i].alpha = (1.0f - life_frac) / 0.1f;  /* quick fade-in */
        else if (life_frac > 0.15f)
            os_labels[i].alpha = 1.0f;                         /* hold */
        else
            os_labels[i].alpha = life_frac / 0.15f;            /* fade out */
        /* No y movement — labels stay fixed */
        if (os_labels[i].life <= 0) os_labels[i].active = 0;
    }

    pthread_mutex_unlock(&vis_mutex);
}

/* DRAW HELPERS*/
static void draw_rounded_badge(float x, float y, float w, float h, Color bg, Color border, const char *text, int font_size, Color text_col) {
    /* Drop shadow */
    DrawRectangleRounded((Rectangle){x+2, y+2, w, h}, 0.35f, 8, (Color){0,0,0,80});
    DrawRectangleRounded((Rectangle){x, y, w, h}, 0.35f, 8, bg);
    DrawRectangleRoundedLines((Rectangle){x, y, w, h}, 0.35f, 8, border);
    DrawText(text, (int)(x + w/2 - MeasureText(text, font_size)/2), (int)(y + h/2 - font_size/2), font_size, text_col);
}

/* Draw a glowing neon line (for roads, pipes, etc.) */
static void draw_neon_line(int x1, int y1, int x2, int y2, Color col, int glow_r) {
    Color g1 = col; g1.a = 40;
    Color g2 = col; g2.a = 90;
    DrawLineEx((Vector2){x1,y1},(Vector2){x2,y2}, glow_r*2.0f, g1);
    DrawLineEx((Vector2){x1,y1},(Vector2){x2,y2}, glow_r*1.0f, g2);
    DrawLineEx((Vector2){x1,y1},(Vector2){x2,y2}, 1.5f, col);
}

/* Draw building with realistic facade */
static void draw_building(int bx, int by, int bw, int bh, Color body, Color accent, int seed, float t, int inter) {
    /* Perspective shadow */
    DrawRectangle(bx+5, by+5, bw, bh, (Color){0,0,0,100});

    /* Building body gradient — darker at bottom */
    for (int row = 0; row < bh; row += 2) {
        float frac = (float)row / bh;
        Color rc = {
            (unsigned char)(body.r * (1.0f - frac*0.25f)),
            (unsigned char)(body.g * (1.0f - frac*0.25f)),
            (unsigned char)(body.b * (1.0f - frac*0.25f)),
            255
        };
        DrawRectangle(bx, by+row, bw, 2, rc);
    }

    /* Facade glass curtain-wall effect (tall buildings) */
    if (bh > 80) {
        for (int col = 0; col < bw; col += 8) {
            DrawLine(bx+col, by, bx+col, by+bh, (Color){255,255,255,12});
        }
    }

    /* Accent top cap */
    DrawRectangle(bx, by, bw, 6, accent);
    DrawRectangle(bx, by+6, bw, 2, (Color){255,255,255,60});

    /* Windows grid */
    int win_w = 6, win_h = 8, win_gap_x = 4, win_gap_y = 6;
    int cols = (bw - 6) / (win_w + win_gap_x);
    int rows = (bh - 16) / (win_h + win_gap_y);
    for (int wr = 0; wr < rows; wr++) {
        for (int wc = 0; wc < cols; wc++) {
            int wx = bx + 3 + wc * (win_w + win_gap_x);
            int wy = by + 10 + wr * (win_h + win_gap_y);
            int wseed = seed + wc*13 + wr*29 + inter*41;
            int lit = ((wseed % 5) != 0);
            if (lit) {
                float flk = 0.75f + 0.25f * sinf(t * (1.2f + (wseed%4)*0.3f) + wseed*0.17f);
                unsigned char br = (unsigned char)(130*flk + 70);
                /* warm yellow or cool blue windows */
                Color wc2 = ((wseed % 6) < 2)
                    ? (Color){br, (unsigned char)(br*0.9f), (unsigned char)(br*0.4f), 240}
                    : (Color){(unsigned char)(br*0.4f), (unsigned char)(br*0.6f), br, 240};
                DrawRectangle(wx, wy, win_w, win_h, wc2);
                /* Window frame highlight */
                DrawLine(wx, wy, wx+win_w, wy, (Color){255,255,255,30});
                DrawLine(wx, wy, wx, wy+win_h, (Color){255,255,255,20});
            } else {
                DrawRectangle(wx, wy, win_w, win_h, (Color){55, 62, 82, 230});
            }
        }
    }

    /* Rooftop features */
    if ((seed % 3) == 0) {
        /* Antenna with blinking light */
        int ax = bx + bw/2;
        DrawLine(ax, by-18, ax, by, (Color){100,110,128,220});
        DrawLine(ax-4, by-10, ax+4, by-10, (Color){100,110,128,180});
        float blink = 0.5f + 0.5f*sinf(t*3.0f + seed);
        DrawCircle(ax, by-18, 3, (Color){255,60,60,(unsigned char)(200*blink)});
        DrawCircle(ax, by-18, 6, (Color){255,40,40,(unsigned char)(60*blink)});
    } else if ((seed % 3) == 1) {
        /* Water tower */
        DrawRectangle(bx+bw/2-8, by-14, 16, 14, (Color){55,65,85,230});
        DrawRectangle(bx+bw/2-6, by-12, 12, 10, (Color){45,120,170,210});
        DrawLine(bx+bw/2-8, by-14, bx+bw/2+8, by-14, (Color){80,90,110,200});
    } else {
        /* HVAC units */
        DrawRectangle(bx+3, by-6, 10, 6, (Color){60,68,80,220});
        DrawRectangle(bx+bw-13, by-6, 10, 6, (Color){60,68,80,220});
    }
}

/*    DRAW UPPER BUILDINGS*/
static void draw_upper_buildings(float t) {
    /* The available vertical band: title bar bottom (50) to road top sidewalk (ROAD_Y-ROAD_HW-14) */
    int zone_top  = 50;
    int zone_bot  = ROAD_Y - ROAD_HW - 14;  /* ~400 */
    int zone_h    = zone_bot - zone_top;     /* ~350 */

      for (int half = 0; half < 2; half++) {
        int hx  = half * (SIM_AREA_W / 2);
        int hw  = SIM_AREA_W / 2;
        int seed_base = 7777 + half * 1111;

        /* Silhouette back-row */
        for (int b = 0; b < 14; b++) {
            int seed  = seed_base + b * 53;
            int bld_w = 20 + (seed % 30);
            int bld_h = 30 + (seed % 70);
            int bld_x = hx + (seed % (hw - bld_w - 2));
            /* Base flush to zone_bot, grows upward */
            int bld_y = zone_bot - bld_h;
            Color sil = (half == 0)
                ? (Color){110, 128, 162, (unsigned char)(120 + b * 5)}
                : (Color){100, 118, 152, (unsigned char)(120 + b * 5)};
            DrawRectangle(bld_x, bld_y, bld_w, bld_h, sil);
            /* Tiny silhouette windows */
            for (int wr = 0; wr < bld_h / 12 - 1; wr++) {
                for (int wc = 0; wc < 3; wc++) {
                    if (((seed + wr * 9 + wc * 13) % 4) != 0) {
                        float flk = 0.5f + 0.5f * sinf(t * (0.9f + (seed % 3) * 0.3f) + wr + wc);
                        DrawRectangle(bld_x + 3 + wc * (bld_w / 3), bld_y + 5 + wr * 11,
                                      bld_w / 3 - 4, 5,
                                      (Color){180, 190, 100, (unsigned char)(80 * flk)});
                    }
                }
            }
        }

        /* Mid-row buildings with colour */
        int mid_cols = 8;
        int mid_pad  = 4;
        int mid_bw   = (hw - mid_pad * (mid_cols + 1)) / mid_cols;
        Color mid_accents[] = {
            {60, 130, 220, 200}, {130, 60, 220, 200}
        };
        Color mid_accent = mid_accents[half];

        for (int b = 0; b < mid_cols; b++) {
            int seed  = seed_base + b * 79 + 300;
            int bld_h = 50 + (seed % 90);
            int bld_x = hx + mid_pad + b * (mid_bw + mid_pad);
            int bld_y = zone_bot - bld_h;
            Color body;
            switch ((b + half * 4) % 6) {
                case 0: body = (Color){155, 168, 195, 235}; break;  /* slate blue-grey */
                case 1: body = (Color){140, 155, 185, 235}; break;  /* steel blue-grey */
                case 2: body = (Color){160, 172, 198, 235}; break;  /* periwinkle grey */
                case 3: body = (Color){148, 162, 192, 235}; break;  /* cool blue-grey */
                case 4: body = (Color){165, 175, 200, 235}; break;  /* mist blue */
                default:body = (Color){152, 165, 190, 235}; break;  /* dusk blue-grey */
            }
            draw_building(bld_x, bld_y, mid_bw, bld_h, body, mid_accent, seed, t, half);
        }

        /* Foreground buildings — tallest, most detailed */
        int fg_cols = 5;
        int fg_pad  = 5;
        int fg_bw   = (hw - fg_pad * (fg_cols + 1)) / fg_cols;
        Color fg_accents[] = {
            {70, 160, 255, 255}, {160, 75, 255, 255}
        };
        Color fg_accent = fg_accents[half];

        for (int b = 0; b < fg_cols; b++) {
            int seed  = seed_base + b * 113 + 600;
            int bld_h = 90 + (seed % 130);
            /* Clamp so building doesn't go above title bar */
            if (bld_h > zone_h - 10) bld_h = zone_h - 10;
            int bld_x = hx + fg_pad + b * (fg_bw + fg_pad);
            int bld_y = zone_bot - bld_h;
            Color body;
            switch ((b + half * 3) % 6) {
                case 0: body = (Color){178, 192, 220, 255}; break;  /* pale sky blue */
                case 1: body = (Color){162, 178, 210, 255}; break;  /* cornflower grey */
                case 2: body = (Color){172, 185, 215, 255}; break;  /* powder blue-grey */
                case 3: body = (Color){185, 198, 222, 255}; break;  /* icy blue */
                case 4: body = (Color){168, 182, 212, 255}; break;  /* steel periwinkle */
                default:body = (Color){175, 190, 218, 255}; break;  /* glacial blue */
            }
            draw_building(bld_x, bld_y, fg_bw, bld_h, body, fg_accent, seed, t, half);

            /* Rooftop glow halo on tallest buildings */
            if (bld_h > 110) {
                Color gc = fg_accent; gc.a = 22;
                DrawRectangle(bld_x - 3, bld_y - 3, fg_bw + 6, 14, gc);
            }
        }

        /* ── Sidewalk strip just above road — grey concrete ── */
        DrawRectangle(hx, zone_bot, hw, 14, (Color){80, 82, 85, 255});
        for (int tx = hx + 4; tx < hx + hw - 4; tx += 18)
            DrawLine(tx, zone_bot, tx, zone_bot + 14, (Color){65, 68, 70, 160});

        /* ── Street lamps on upper side (upside-down style on curb) ── */
        for (int p = 0; p < 5; p++) {
            int lpx = hx + 30 + p * (hw / 5);
            int lpy = zone_bot + 5;
            DrawRectangle(lpx - 1, lpy - 30, 3, 30, (Color){75, 82, 96, 230});
            DrawLine(lpx + 1, lpy - 30, lpx + 14, lpy - 22, (Color){75, 82, 96, 210});
            DrawRectangleRounded((Rectangle){lpx + 10, lpy - 26, 14, 8}, 0.4f, 4, (Color){60, 65, 78, 240});
            float glow = 0.55f + 0.45f * sinf(t * 1.8f + p + half * 2.5f);
            DrawCircle(lpx + 17, lpy - 22, 4,  (Color){255, 225, 140, (unsigned char)(200 * glow)});
            DrawCircle(lpx + 17, lpy - 22, 10, (Color){255, 215, 110, (unsigned char)(50 * glow)});
            DrawCircle(lpx + 17, lpy - 22, 20, (Color){255, 200, 80,  (unsigned char)(15 * glow)});
        }
    }

    /* ── Neon divider line at road-top edge ── */
    draw_neon_line(0, zone_bot, SIM_AREA_W, zone_bot, (Color){55, 90, 160, 180}, 2);
}

static void draw_city_district(int inter, float t) {
    int bx = (inter == INTERSECTION_F10) ? DIST_F10_X : DIST_F11_X;
    int by = DIST_F10_Y;
    int bw = DIST_F10_W;
    int bh = DIST_F10_H;

    /* ── Green ground fill for district zone ── */
    for (int row = 0; row < 120; row++) {
        float frac = (float)row / 120.0f;
        /* Lighter green at top of district, darkening slightly */
        Color sky = (Color){
            (unsigned char)(30 + frac * 10),
            (unsigned char)(80 + frac * 20),
            (unsigned char)(30 + frac * 10), 255};
        DrawLine(bx, by+row, bx+bw, by+row, sky);
    }

    /* ── Ground fill ── */
    DrawRectangle(bx, by+120, bw, bh-120, (inter==0)
        ? (Color){28, 72, 28, 255} : (Color){26, 68, 26, 255});

    /* ── Stars / ambient particles ── */
    for (int s = 0; s < 18; s++) {
        int seed = inter*999 + s*37;
        int sx = bx + (seed % bw);
        int sy = by + (seed % 100);
        float twinkle = 0.4f + 0.6f*sinf(t*1.3f + s*1.7f);
        DrawPixel(sx, sy, (Color){220,230,255,(unsigned char)(120*twinkle)});
    }

    /* ── DEEP BACKGROUND BUILDINGS (silhouette layer) ── */
    int seed_base = inter * 1000;
    for (int b = 0; b < 16; b++) {
        int seed  = seed_base + b * 41;
        int bld_w = 18 + (seed % 26);
        int bld_h = 40 + (seed % 80);
        int bld_x = bx + (seed % (bw - bld_w - 4));
        int bld_y = by + 100 - bld_h;
        Color sil = (inter == 0)
            ? (Color){105, 122, 158, (unsigned char)(140 + b*4)}
            : (Color){95, 112, 148, (unsigned char)(140 + b*4)};
        DrawRectangle(bld_x, bld_y, bld_w, bld_h, sil);
        /* silhouette windows */
        for (int wr = 1; wr < bld_h/10; wr++) {
            for (int wc = 0; wc < 3; wc++) {
                if (((seed+wr*7+wc*11)%4)!=0) {
                    float flk = 0.6f+0.4f*sinf(t*(0.8f+(seed%3)*0.4f)+wr+wc);
                    DrawRectangle(bld_x+3+wc*(bld_w/3), bld_y+4+wr*9, bld_w/3-4, 5,
                        (Color){180,190,100,(unsigned char)(100*flk)});
                }
            }
        }
    }

    int fld_base_y = by + bh - 110;

    /* ── MID LAYER BUILDINGS ── */
    int mid_cols = 9;
    int mid_pad = 4;
    int mid_bw = (bw - mid_pad*(mid_cols+1)) / mid_cols;
    for (int b = 0; b < mid_cols; b++) {
        int seed = seed_base + b*71 + 200;
        int bld_h = 45 + (seed%80);
        int bld_x = bx + mid_pad + b*(mid_bw+mid_pad);
        int bld_y = fld_base_y - bld_h - 30;
        Color accent = (inter==0)
            ? (Color){90, 140, 220, 220} : (Color){160, 90, 220, 220};
        Color body;
        switch((b+inter*4)%6){
            case 0: body=(Color){158, 172, 205, 240}; break;  /* slate blue-grey */
            case 1: body=(Color){145, 160, 195, 240}; break;  /* steel blue-grey */
            case 2: body=(Color){162, 175, 208, 240}; break;  /* periwinkle grey */
            case 3: body=(Color){150, 165, 198, 240}; break;  /* cool blue-grey */
            case 4: body=(Color){168, 178, 210, 240}; break;  /* mist blue */
            default:body=(Color){155, 168, 200, 240}; break;  /* dusk blue-grey */
        }
        draw_building(bld_x, bld_y, mid_bw, bld_h, body, accent, seed, t, inter);
    }

    /* ── FOREGROUND BUILDINGS (main, most detailed) ── */
    int fld_cols = 6;
    int fld_pad  = 5;
    int fld_bw   = (bw - fld_pad*(fld_cols+1)) / fld_cols;
    for (int b = 0; b < fld_cols; b++) {
        int seed  = seed_base + b*97 + 500;
        int bld_h = 80 + (seed%110);
        int bld_x = bx + fld_pad + b*(fld_bw+fld_pad);
        int bld_y = fld_base_y - bld_h;

        Color accent = (inter==0)
            ? (Color){55,145,235,255} : (Color){145,65,235,255};
        Color body;
        switch((b+inter*3)%6){
            case 0: body=(Color){180, 195, 225, 255}; break;  /* pale sky blue */
            case 1: body=(Color){165, 180, 215, 255}; break;  /* cornflower grey */
            case 2: body=(Color){175, 188, 218, 255}; break;  /* powder blue-grey */
            case 3: body=(Color){188, 202, 228, 255}; break;  /* icy blue */
            case 4: body=(Color){170, 185, 215, 255}; break;  /* steel periwinkle */
            default:body=(Color){178, 192, 222, 255}; break;  /* glacial blue */
        }
        draw_building(bld_x, bld_y, fld_bw, bld_h, body, accent, seed, t, inter);

        /* Glow at top of tall buildings */
        if (bld_h > 100) {
            Color gc = accent; gc.a = 25;
            DrawRectangle(bld_x-3, bld_y-3, fld_bw+6, 12, gc);
        }
    }

    /* ── Ground pavement — dark grey concrete ── */
    DrawRectangle(bx, fld_base_y, bw, 110, (Color){48, 52, 55, 255});
    /* Pavement tiles */
    for (int tx = bx; tx < bx+bw; tx += 20) {
        DrawLine(tx, fld_base_y, tx, fld_base_y+10, (Color){38,40,42,200});
    }
    /* Curb */
    DrawRectangle(bx, fld_base_y, bw, 4, (Color){65, 68, 72, 255});
    DrawLine(bx, fld_base_y+4, bx+bw, fld_base_y+4, (Color){80,84,88,200});

    /* ── Sidewalk markings ── */
    for (int tx = bx+5; tx < bx+bw-10; tx += 14) {
        DrawRectangle(tx, fld_base_y+5, 8, 4, (Color){42,45,48,220});
    }

    /* ── Inner road within district (driveways) — dark grey ── */
    DrawRectangle(bx, fld_base_y+20, bw, 18, (Color){30, 32, 34, 255});
    /* Dashed centerline */
    for (int tx = bx; tx < bx+bw; tx+=22)
        DrawRectangle(tx, fld_base_y+28, 12, 2, (Color){220, 200, 60, 160});

    /* ── Parked cars along district road ── */
    for (int p = 0; p < 5; p++) {
        int pcx = bx + 15 + p*(bw/5);
        int pcy = fld_base_y + 21;
        Color pc = (Color){(unsigned char)(50+p*20),(unsigned char)(80+p*10),(unsigned char)(120+p*15),255};
        DrawRectangleRounded((Rectangle){pcx, pcy, 22, 12}, 0.2f, 4, pc);
        DrawRectangle(pcx+3, pcy+1, 16, 5, (Color){140,195,230,160});
        DrawCircle(pcx+4, pcy+12, 3, (Color){35,35,35,220});
        DrawCircle(pcx+18, pcy+12, 3, (Color){35,35,35,220});
    }

    /* ── Street lamps — detailed ── */
    for (int p = 0; p < 6; p++) {
        int lpx = bx + 25 + p*(bw/6);
        int lpy = fld_base_y + 5;
        /* Pole */
        DrawRectangle(lpx-1, lpy, 3, 38, (Color){75,82,96,230});
        /* Arm */
        DrawLine(lpx+1, lpy, lpx+14, lpy-8, (Color){75,82,96,210});
        /* Lamp housing */
        DrawRectangleRounded((Rectangle){lpx+10, lpy-12, 14, 8}, 0.4f, 4, (Color){60,65,78,240});
        float glow = 0.55f + 0.45f*sinf(t*1.8f + p);
        /* Warm lamp glow */
        DrawCircle(lpx+17, lpy-8, 4, (Color){255,225,140,(unsigned char)(200*glow)});
        DrawCircle(lpx+17, lpy-8, 10, (Color){255,215,110,(unsigned char)(55*glow)});
        DrawCircle(lpx+17, lpy-8, 20, (Color){255,200,80,(unsigned char)(18*glow)});
        /* Light cone on ground */
        DrawTriangle(
            (Vector2){lpx+12, lpy-4},
            (Vector2){lpx+22, lpy-4},
            (Vector2){lpx+26, lpy+15},
            (Color){255,225,140,(unsigned char)(12*glow)}
        );
    }

    /* Thread pool indicator  */
    /* Shows active vehicle threads */
    {
        int tx = bx + bw - 115, ty = by + bh - 45;
        DrawRectangleRounded((Rectangle){tx-4, ty-4, 110, 38}, 0.2f, 6, (Color){10,12,22,210});
        DrawRectangleRoundedLines((Rectangle){tx-4, ty-4, 110, 38}, 0.2f, 6,
            (inter==0)?(Color){55,140,220,160}:(Color){140,60,220,160});
        DrawText("pthread_t threads:", tx, ty, 7, (Color){140,160,200,200});
        /* Thread count dots */
        int active_count = 0;
        for (int i = 0; i < vis_veh_count; i++)
            if (vis_vehicles[i].active && vis_vehicles[i].intersection == inter) active_count++;
        for (int i = 0; i < 8; i++) {
            Color dc = (i < active_count)
                ? (inter==0?(Color){55,200,120,230}:(Color){200,100,255,230})
                : (Color){30,35,50,180};
            DrawCircle(tx + 5 + i*13, ty+22, 4, dc);
        }
    }

    /* ── Population badge with civic icon ── */
    Color badge_col = (inter == INTERSECTION_F10)
        ? (Color){45, 110, 210, 235} : (Color){130, 55, 210, 235};
    Color badge_bg  = (Color){8, 10, 20, 230};

    const char *district_name = (inter == INTERSECTION_F10) ? "F10  DISTRICT" : "F11  DISTRICT";
    const char *district_sub  = (inter == INTERSECTION_F10) ? "NORTH SECTOR"  : "EAST SECTOR";
    const char *dist_role     = (inter == INTERSECTION_F10) ? "PROCESS: pid_f10" : "PROCESS: pid_f11";
    int lbl_w = MeasureText(district_name, 13) + 24;

    DrawRectangleRounded((Rectangle){bx+6, by+6, lbl_w, 52}, 0.25f, 8, badge_bg);
    DrawRectangleRoundedLines((Rectangle){bx+6, by+6, lbl_w, 52}, 0.25f, 8, badge_col);
    /* Colored left bar */
    DrawRectangleRounded((Rectangle){bx+6, by+6, 5, 52}, 0.1f, 4, badge_col);
    DrawText(district_name, bx+18, by+10, 13, badge_col);
    DrawText(district_sub,  bx+18, by+26, 9,  (Color){140,165,210,190});
    DrawText(dist_role,     bx+18, by+38, 8,  (Color){100,140,200,170});

    /* Population counter with icon */
    char pop_str[40];
    int base_pop = 14000 + inter*3200;
    /* Simulate population activity */
    int dyn_pop = base_pop + (int)(sinf(t*0.3f)*120);
    snprintf(pop_str, sizeof(pop_str), "POP: %d", dyn_pop);
    int pw = MeasureText(pop_str, 9);
    DrawRectangleRounded((Rectangle){bx+bw-pw-20, by+8, pw+16, 20}, 0.4f, 6, (Color){10,12,24,210});
    DrawRectangleRoundedLines((Rectangle){bx+bw-pw-20, by+8, pw+16, 20}, 0.4f, 6, badge_col);
    DrawText(pop_str, bx+bw-pw-12, by+12, 9, badge_col);

    /* Fork() / process badge */
    const char *fork_lbl = (inter==0) ? "fork(): F10" : "fork(): F11";
    int flw = MeasureText(fork_lbl, 8);
    DrawRectangleRounded((Rectangle){bx+bw-flw-20, by+32, flw+16, 18}, 0.4f, 6, (Color){10,12,24,210});
    DrawRectangleRoundedLines((Rectangle){bx+bw-flw-20, by+32, flw+16, 18}, 0.4f, 6, (Color){100,220,160,180});
    DrawText(fork_lbl, bx+bw-flw-12, by+35, 8, (Color){100,220,160,220});

    /* ── Divider line at district top — glowing ── */
    draw_neon_line(bx, by, bx+bw, by, badge_col, 2);
    /* Vertical divider between districts */
    if (inter == INTERSECTION_F10)
        DrawLine(bx+bw, by, bx+bw, by+bh, (Color){50,55,75,200});
}

/* 
   DRAW ROADS */
static void draw_roads(float t) {
    Color road      = (Color){20, 20, 22, 255};    /* near-black asphalt */
    Color road_dark = (Color){14, 14, 16, 255};    /* darker texture strips */
    Color sidewalk  = (Color){80, 82, 85, 255};    /* medium grey concrete */
    Color curb_lo   = (Color){100, 102, 106, 255}; /* light curb edge */
    Color curb_hi   = (Color){110, 113, 118, 255}; /* bright curb highlight */
    Color yell_dash = (Color){240, 210, 40, 230};  /* bright yellow lane marks */
    Color wht_dash  = (Color){230, 232, 235, 180}; /* white lane dividers */

    /* SIDEWALKS  */
    /* Horizontal road sidewalks — lower side WIDER (24px) to match upper district's feel */
    DrawRectangle(0, ROAD_Y - ROAD_HW - 14, SIM_AREA_W, 14, sidewalk);
    DrawRectangle(0, ROAD_Y + ROAD_HW,      SIM_AREA_W, 24, sidewalk);  /* wider lower sidewalk */
    /* Sidewalk tile lines */
    for (int tx = 0; tx < SIM_AREA_W; tx += 18) {
        DrawLine(tx, ROAD_Y-ROAD_HW-14, tx, ROAD_Y-ROAD_HW, (Color){38,42,52,160});
        DrawLine(tx, ROAD_Y+ROAD_HW,    tx, ROAD_Y+ROAD_HW+24, (Color){38,42,52,160});
    }
    /* Curb edges */
    DrawRectangle(0, ROAD_Y-ROAD_HW-3, SIM_AREA_W, 3, curb_lo);
    DrawRectangle(0, ROAD_Y+ROAD_HW,   SIM_AREA_W, 4, curb_hi);   /* thicker lower curb */

    /* Vertical road sidewalks */
    DrawRectangle(F10_CX-ROAD_HW-14, 50, 14, DIST_F10_Y-50, sidewalk);
    DrawRectangle(F10_CX+ROAD_HW,    50, 14, DIST_F10_Y-50, sidewalk);
    DrawRectangle(F11_CX-ROAD_HW-14, 50, 14, DIST_F11_Y-50, sidewalk);
    DrawRectangle(F11_CX+ROAD_HW,    50, 14, DIST_F11_Y-50, sidewalk);
    /* Sidewalk tile lines on vertical */
    for (int ty = 50; ty < DIST_F10_Y; ty += 18) {
        DrawLine(F10_CX-ROAD_HW-14, ty, F10_CX-ROAD_HW, ty, (Color){38,42,52,140});
        DrawLine(F10_CX+ROAD_HW,    ty, F10_CX+ROAD_HW+14, ty, (Color){38,42,52,140});
        DrawLine(F11_CX-ROAD_HW-14, ty, F11_CX-ROAD_HW, ty, (Color){38,42,52,140});
        DrawLine(F11_CX+ROAD_HW,    ty, F11_CX+ROAD_HW+14, ty, (Color){38,42,52,140});
    }

    /* ── HORIZONTAL MAIN ROAD — 4 lanes ── */
    /* Road body */
    DrawRectangle(0, ROAD_Y-ROAD_HW, SIM_AREA_W, ROAD_HW*2, road);
    /* Subtle texture variation */
    for (int x = 0; x < SIM_AREA_W; x += 60) {
        DrawRectangle(x, ROAD_Y-ROAD_HW, 58, ROAD_HW*2, road_dark);
    }
    /* Re-draw solid over texture (keep clean) */
    DrawRectangle(0, ROAD_Y-ROAD_HW, SIM_AREA_W, ROAD_HW*2, (Color){22,24,32,200});

    /* Road edge curb lines */
    DrawRectangle(0, ROAD_Y-ROAD_HW-2, SIM_AREA_W, 2, curb_hi);
    DrawRectangle(0, ROAD_Y+ROAD_HW,   SIM_AREA_W, 2, curb_lo);

    /* ── VERTICAL ROADS ── */
    DrawRectangle(F10_CX-ROAD_HW, 50, ROAD_HW*2, DIST_F10_Y-50, road);
    DrawRectangle(F11_CX-ROAD_HW, 50, ROAD_HW*2, DIST_F11_Y-50, road);
    /* Curb lines */
    DrawRectangle(F10_CX-ROAD_HW-2, 50, 2, DIST_F10_Y-50, curb_hi);
    DrawRectangle(F10_CX+ROAD_HW,   50, 2, DIST_F10_Y-50, curb_lo);
    DrawRectangle(F11_CX-ROAD_HW-2, 50, 2, DIST_F11_Y-50, curb_hi);
    DrawRectangle(F11_CX+ROAD_HW,   50, 2, DIST_F11_Y-50, curb_lo);

    /* ── CENTRE DASHES — horizontal road (double yellow) ── */
    for (int x = 0; x < SIM_AREA_W; x += 30) {
        int skip10 = (x > F10_CX-ROAD_HW-2 && x < F10_CX+ROAD_HW+2);
        int skip11 = (x > F11_CX-ROAD_HW-2 && x < F11_CX+ROAD_HW+2);
        if (!skip10 && !skip11) {
            DrawRectangle(x,    ROAD_Y-2, 18, 2, yell_dash);
            DrawRectangle(x,    ROAD_Y+1, 18, 1, (Color){200,175,35,160});
        }
    }

    /* ── LANE DIVIDERS — white dashed ── */
    for (int x = 0; x < SIM_AREA_W; x += 26) {
        int skip10 = (x > F10_CX-ROAD_HW-2 && x < F10_CX+ROAD_HW+2);
        int skip11 = (x > F11_CX-ROAD_HW-2 && x < F11_CX+ROAD_HW+2);
        if (!skip10 && !skip11) {
            DrawRectangle(x, ROAD_Y-ROAD_HW+LANE_W,   14, 1, wht_dash);
            DrawRectangle(x, ROAD_Y+ROAD_HW-LANE_W-1, 14, 1, wht_dash);
        }
    }

    /* ── CENTRE DASHES — vertical roads ── */
    for (int y = 50; y < DIST_F10_Y; y += 30) {
        int skiph = (y > ROAD_Y-ROAD_HW-2 && y < ROAD_Y+ROAD_HW+2);
        if (!skiph) {
            DrawRectangle(F10_CX-1, y, 1, 18, yell_dash);
            DrawRectangle(F10_CX+1, y, 1, 18, yell_dash);
            DrawRectangle(F11_CX-1, y, 1, 18, yell_dash);
            DrawRectangle(F11_CX+1, y, 1, 18, yell_dash);
        }
    }

    /* ── LANE DIVIDERS — vertical roads ── */
    for (int y = 50; y < DIST_F10_Y; y += 26) {
        int skiph = (y > ROAD_Y-ROAD_HW-2 && y < ROAD_Y+ROAD_HW+2);
        if (!skiph) {
            DrawRectangle(F10_CX-ROAD_HW+LANE_W, y, 1, 14, wht_dash);
            DrawRectangle(F10_CX+ROAD_HW-LANE_W, y, 1, 14, wht_dash);
            DrawRectangle(F11_CX-ROAD_HW+LANE_W, y, 1, 14, wht_dash);
            DrawRectangle(F11_CX+ROAD_HW-LANE_W, y, 1, 14, wht_dash);
        }
    }

    /* ── STOP LINES (thick, bold) ── */
    Color stop_wht = (Color){245, 245, 250, 220};
    /* F10 */
    DrawRectangle(F10_CX-ROAD_HW-4, ROAD_Y-ROAD_HW, 6, ROAD_HW, stop_wht);
    DrawRectangle(F10_CX+ROAD_HW-2, ROAD_Y,          6, ROAD_HW, stop_wht);
    DrawRectangle(F10_CX-ROAD_HW,   ROAD_Y-ROAD_HW-4, ROAD_HW, 6, stop_wht);
    DrawRectangle(F10_CX,           ROAD_Y+ROAD_HW-2,  ROAD_HW, 6, stop_wht);
    /* F11 */
    DrawRectangle(F11_CX-ROAD_HW-4, ROAD_Y-ROAD_HW, 6, ROAD_HW, stop_wht);
    DrawRectangle(F11_CX+ROAD_HW-2, ROAD_Y,          6, ROAD_HW, stop_wht);
    DrawRectangle(F11_CX-ROAD_HW,   ROAD_Y-ROAD_HW-4, ROAD_HW, 6, stop_wht);
    DrawRectangle(F11_CX,           ROAD_Y+ROAD_HW-2,  ROAD_HW, 6, stop_wht);

    /* ── ZEBRA CROSSINGS ── */
    Color zebra = (Color){195,200,210,90};
    for (int i = 0; i < 5; i++) {
        /* F10 */
        DrawRectangle(F10_CX-ROAD_HW+2+i*12, ROAD_Y-ROAD_HW-12, 9, 12, zebra);
        DrawRectangle(F10_CX-ROAD_HW+2+i*12, ROAD_Y+ROAD_HW,     9, 12, zebra);
        DrawRectangle(F10_CX-ROAD_HW-12, ROAD_Y-ROAD_HW+2+i*12, 12, 9, zebra);
        DrawRectangle(F10_CX+ROAD_HW,    ROAD_Y-ROAD_HW+2+i*12, 12, 9, zebra);
        /* F11 */
        DrawRectangle(F11_CX-ROAD_HW+2+i*12, ROAD_Y-ROAD_HW-12, 9, 12, zebra);
        DrawRectangle(F11_CX-ROAD_HW+2+i*12, ROAD_Y+ROAD_HW,     9, 12, zebra);
        DrawRectangle(F11_CX-ROAD_HW-12, ROAD_Y-ROAD_HW+2+i*12, 12, 9, zebra);
        DrawRectangle(F11_CX+ROAD_HW,    ROAD_Y-ROAD_HW+2+i*12, 12, 9, zebra);
    }

    /* ── ANIMATED TRAFFIC FLOW DOTS (show direction) ── */
    float flow_off = fmodf(t * 45.0f, 60.0f);
    for (int x = 0; x < SIM_AREA_W; x += 60) {
        int fx = (int)(x + flow_off) % SIM_AREA_W;
        int skip10 = (fx > F10_CX-ROAD_HW-2 && fx < F10_CX+ROAD_HW+2);
        int skip11 = (fx > F11_CX-ROAD_HW-2 && fx < F11_CX+ROAD_HW+2);
        if (!skip10 && !skip11) {
            DrawCircle(fx, ROAD_Y-20, 2, (Color){80,120,200,100});
            DrawCircle(fx, ROAD_Y+20, 2, (Color){80,120,200,100});
        }
    }
    /* Vertical flow dots */
    float vflow = fmodf(t * 40.0f, 50.0f);
    for (int y = 50; y < DIST_F10_Y; y += 50) {
        int fy = (int)(y + vflow) % DIST_F10_Y;
        int skiph = (fy > ROAD_Y-ROAD_HW-2 && fy < ROAD_Y+ROAD_HW+2);
        if (!skiph && fy > 50) {
            DrawCircle(F10_CX+18, fy, 2, (Color){80,150,200,80});
            DrawCircle(F11_CX+18, fy, 2, (Color){80,150,200,80});
        }
    }

    /* ── ROAD NAME SIGNS ── */
    int mid_x = (F10_CX + F11_CX) / 2;
    /* Sign panel for MAIN AVE */
    DrawRectangleRounded((Rectangle){mid_x-38, ROAD_Y-ROAD_HW-26, 76, 16}, 0.35f, 4,
                         (Color){28,36,58,200});
    DrawRectangleRoundedLines((Rectangle){mid_x-38, ROAD_Y-ROAD_HW-26, 76, 16}, 0.35f, 4,
                              (Color){80,100,160,180});
    DrawText("MAIN AVE", mid_x-MeasureText("MAIN AVE",9)/2, ROAD_Y-ROAD_HW-23, 9,
             (Color){160,180,220,220});

    DrawRectangleRounded((Rectangle){F10_CX-20, 50, 40, 14}, 0.35f, 4, (Color){24,32,52,200});
    DrawText("F10 ST", F10_CX-MeasureText("F10 ST",8)/2, 53, 8, (Color){100,140,210,200});
    DrawRectangleRounded((Rectangle){F11_CX-20, 50, 40, 14}, 0.35f, 4, (Color){24,32,52,200});
    DrawText("F11 ST", F11_CX-MeasureText("F11 ST",8)/2, 53, 8, (Color){100,140,210,200});
}

/* 
   DRAW TRAFFIC LIGHT POLE 
 */
static void draw_traffic_light_pole(int px, int py, int is_green, float t) {
    /* Pole base foundation */
    DrawRectangle(px-6, py-2, 12, 6, (Color){40,44,54,255});
    DrawRectangleRoundedLines((Rectangle){px-6, py-2, 12, 6}, 0.3f, 4, (Color){65,70,85,220});

    /* Pole body (with subtle 3D shading) */
    DrawRectangle(px-3, py-74, 7, 72, (Color){55,60,72,255});
    DrawRectangle(px-3, py-74, 2, 72, (Color){80,86,100,200}); /* left highlight */
    DrawRectangle(px+3, py-74, 1, 72, (Color){30,33,42,255});  /* right shadow */

    /* Cross arm */
    DrawRectangle(px-3, py-74, 24, 5, (Color){55,60,72,255});
    DrawRectangle(px-3, py-74, 24, 2, (Color){80,86,100,160});

    /* Housing box (with inner shadow) */
    int hx = px+16, hy = py-82;
    DrawRectangleRounded((Rectangle){hx-1, hy-1, 22, 44}, 0.25f, 8, (Color){10,11,18,255});
    DrawRectangleRounded((Rectangle){hx, hy, 20, 42}, 0.25f, 8, (Color){20,22,30,255});
    DrawRectangleRoundedLines((Rectangle){hx, hy, 20, 42}, 0.25f, 8, (Color){60,66,82,230});
    /* Housing highlight */
    DrawLine(hx+1, hy+1, hx+1, hy+40, (Color){255,255,255,20});
    DrawLine(hx+1, hy+1, hx+18, hy+1, (Color){255,255,255,15});

    int lx = hx + 10;
    /* Amber lens (middle) — always dim */
    DrawCircle(lx, hy+21, 6, (Color){40,35,8,255});
    DrawCircle(lx, hy+21, 4, (Color){55,48,12,255});

    /* ── RED BULB ── */
    int ly_r = hy+8;
    if (!is_green) {
        float pulse = 0.6f + 0.4f*sinf(t*7.5f);
        /* Outer glow halos */
        DrawCircle(lx, ly_r, 22, (Color){255,20,20,(unsigned char)(12*pulse)});
        DrawCircle(lx, ly_r, 16, (Color){255,30,20,(unsigned char)(28*pulse)});
        DrawCircle(lx, ly_r, 11, (Color){255,40,25,(unsigned char)(60*pulse)});
        /* Lens */
        DrawCircle(lx, ly_r, 7, (Color){255,50,40,(unsigned char)(235*pulse)});
        /* Specular highlight */
        DrawCircle(lx-2, ly_r-2, 2, (Color){255,200,200,(unsigned char)(180*pulse)});
    } else {
        DrawCircle(lx, ly_r, 7, (Color){45,15,15,255});
        DrawCircle(lx-2, ly_r-2, 2, (Color){70,25,25,255});
    }

    /* ── GREEN BULB ── */
    int ly_g = hy+35;
    if (is_green) {
        float pulse = 0.65f+0.35f*sinf(t*5.0f);
        /* Outer glow halos */
        DrawCircle(lx, ly_g, 22, (Color){20,220,50,(unsigned char)(10*pulse)});
        DrawCircle(lx, ly_g, 16, (Color){20,230,55,(unsigned char)(25*pulse)});
        DrawCircle(lx, ly_g, 11, (Color){25,235,60,(unsigned char)(55*pulse)});
        /* Lens */
        DrawCircle(lx, ly_g, 7, (Color){30,245,70,(unsigned char)(235*pulse)});
        /* Specular */
        DrawCircle(lx-2, ly_g-2, 2, (Color){200,255,210,(unsigned char)(180*pulse)});
    } else {
        DrawCircle(lx, ly_g, 7, (Color){15,45,20,255});
        DrawCircle(lx-2, ly_g-2, 2, (Color){20,60,25,255});
    }
}

/*  DRAW INTERSECTION*/
static void draw_intersection(int inter, float t) {
    int cx = (inter == INTERSECTION_F10) ? F10_CX : F11_CX;
    int cy = ROAD_Y;
    int is_green = (inter == INTERSECTION_F10) ? vis_light_f10 : vis_light_f11;
    float flash  = vis_inter_flash[inter];

    Color road_col  = (Color){18, 18, 20, 255};
    Color road_col2 = (Color){14, 14, 16, 255};

    /* ── Asphalt box (intersection pavement) ── */
    DrawRectangle(cx-ROAD_HW, cy-ROAD_HW, ROAD_HW*2, ROAD_HW*2, road_col);
    /* Subtle grid texture */
    for (int gx = cx-ROAD_HW; gx < cx+ROAD_HW; gx += 12)
        DrawLine(gx, cy-ROAD_HW, gx, cy+ROAD_HW, (Color){12,12,14,120});
    for (int gy = cy-ROAD_HW; gy < cy+ROAD_HW; gy += 12)
        DrawLine(cx-ROAD_HW, gy, cx+ROAD_HW, gy, (Color){12,12,14,120});

    /* ── Corner kerb islands ── */
    int kw = 12;
    Color kerb_base = (Color){75, 78, 82, 255};
    Color kerb_edge = (Color){95, 98, 104, 255};
    /* NW corner */
    DrawRectangle(cx-ROAD_HW-kw, cy-ROAD_HW-kw, kw, kw, kerb_base);
    DrawLine(cx-ROAD_HW-kw, cy-ROAD_HW, cx-ROAD_HW, cy-ROAD_HW, kerb_edge);
    DrawLine(cx-ROAD_HW, cy-ROAD_HW-kw, cx-ROAD_HW, cy-ROAD_HW, kerb_edge);
    /* NE corner */
    DrawRectangle(cx+ROAD_HW, cy-ROAD_HW-kw, kw, kw, kerb_base);
    DrawLine(cx+ROAD_HW, cy-ROAD_HW, cx+ROAD_HW+kw, cy-ROAD_HW, kerb_edge);
    DrawLine(cx+ROAD_HW, cy-ROAD_HW-kw, cx+ROAD_HW, cy-ROAD_HW, kerb_edge);
    /* SW corner */
    DrawRectangle(cx-ROAD_HW-kw, cy+ROAD_HW, kw, kw, kerb_base);
    DrawLine(cx-ROAD_HW-kw, cy+ROAD_HW, cx-ROAD_HW, cy+ROAD_HW, kerb_edge);
    DrawLine(cx-ROAD_HW, cy+ROAD_HW, cx-ROAD_HW, cy+ROAD_HW+kw, kerb_edge);
    /* SE corner */
    DrawRectangle(cx+ROAD_HW, cy+ROAD_HW, kw, kw, kerb_base);
    DrawLine(cx+ROAD_HW, cy+ROAD_HW, cx+ROAD_HW+kw, cy+ROAD_HW, kerb_edge);
    DrawLine(cx+ROAD_HW, cy+ROAD_HW, cx+ROAD_HW, cy+ROAD_HW+kw, kerb_edge);

    /* ── Yellow box junction with animated diagonal hatching ── */
    Color box_col = is_green ? (Color){55,95,45,55} : (Color){95,45,45,55};
    DrawRectangle(cx-ROAD_HW+4, cy-ROAD_HW+4, ROAD_HW*2-8, ROAD_HW*2-8, box_col);

    /* Animated hatching */
    float hatch_off = fmodf(t * 8.0f, 14.0f);
    for (int s = -ROAD_HW*2; s < ROAD_HW*2; s += 14) {
        int sx1 = cx-ROAD_HW+4, sy1 = cy-ROAD_HW+4+(int)(s+hatch_off);
        int sx2 = cx-ROAD_HW+4+(ROAD_HW*2-8), sy2 = cy-ROAD_HW+4+s+(int)hatch_off+(ROAD_HW*2-8);
        DrawLine(sx1, sy1, sx2, sy2, (Color){220,195,45,35});
    }

    /* Yellow box border */
    DrawRectangleLines(cx-ROAD_HW+4, cy-ROAD_HW+4, ROAD_HW*2-8, ROAD_HW*2-8,
                       (Color){210,185,40,100});

    /* ── Directional arrows in lanes ── */
    /* Straight ahead arrow on E-W road */
    {
        int ax = cx-6, ay = cy-6;
        /* Arrow shaft */
        DrawRectangle(ax, ay, 12, 12, (Color){255,255,255,25});
        /* Mini arrow hint */
        DrawLine(cx, cy-10, cx, cy+10, (Color){255,255,255,30});
        DrawLine(cx-5, cy-4, cx, cy-10, (Color){255,255,255,30});
        DrawLine(cx+5, cy-4, cx, cy-10, (Color){255,255,255,30});
    }

    /* ── Traffic light poles (all 4 corners) ── */
    draw_traffic_light_pole(cx-ROAD_HW-4, cy-ROAD_HW+10, is_green, t);
    draw_traffic_light_pole(cx+ROAD_HW+4, cy+ROAD_HW-10, is_green, t);
    draw_traffic_light_pole(cx-ROAD_HW-4, cy+ROAD_HW-10, is_green, t);
    draw_traffic_light_pole(cx+ROAD_HW+4, cy-ROAD_HW+10, is_green, t);

    /* ── Crossing event flash ── */
    if (flash > 0) {
        unsigned char alpha = (unsigned char)(110 * flash / 2.0f);
        DrawRectangle(cx-ROAD_HW, cy-ROAD_HW, ROAD_HW*2, ROAD_HW*2, (Color){255,225,80,alpha});
        float ring_r = ROAD_HW*2.4f + (1.0f-flash)*24.0f;
        DrawCircleLines(cx, cy, ring_r, (Color){255,220,60,(unsigned char)(alpha*0.8f)});
        DrawCircleLines(cx, cy, ring_r+8, (Color){255,200,40,(unsigned char)(alpha*0.4f)});
    }

    /* ── State label badge above intersection ── */
    const char *label  = (inter == INTERSECTION_F10) ? "F10" : "F11";
    const char *sublbl = (inter == INTERSECTION_F10) ? "INTERSECTION" : "INTERSECTION";
    Color badge_col = is_green ? (Color){28,85,42,230} : (Color){85,22,22,230};
    Color txt_col   = is_green ? (Color){75,245,100,255} : (Color){255,75,75,255};
    Color brd_col   = is_green ? (Color){55,200,80,210} : (Color){220,55,55,210};

    int lw = MeasureText(label, 15);
    int badge_x = cx-lw/2-14, badge_y = cy-ROAD_HW-50;
    /* Shadow */
    DrawRectangleRounded((Rectangle){badge_x+2, badge_y+2, lw+28, 34}, 0.4f, 8, (Color){0,0,0,100});
    DrawRectangleRounded((Rectangle){badge_x, badge_y, lw+28, 34}, 0.4f, 8, badge_col);
    DrawRectangleRoundedLines((Rectangle){badge_x, badge_y, lw+28, 34}, 0.4f, 8, brd_col);
    DrawText(label,  cx-lw/2, badge_y+4,  15, txt_col);
    DrawText(sublbl, cx-MeasureText(sublbl,7)/2, badge_y+22, 7, (Color){brd_col.r,brd_col.g,brd_col.b,180});

    /* ── GO / STOP pill below intersection ── */
    const char *state_lbl = is_green ? "  GO  " : " STOP ";
    Color sc  = is_green ? (Color){55,230,75,255} : (Color){255,55,55,255};
    Color sbg = is_green ? (Color){18,65,28,220}  : (Color){65,18,18,220};
    int sw = MeasureText(state_lbl, 10);
    DrawRectangleRounded((Rectangle){cx-sw/2-4, cy+ROAD_HW+kw+4, sw+8, 16}, 0.5f, 6, sbg);
    DrawRectangleRoundedLines((Rectangle){cx-sw/2-4, cy+ROAD_HW+kw+4, sw+8, 16}, 0.5f, 6, sc);
    DrawText(state_lbl, cx-sw/2, cy+ROAD_HW+kw+7, 10, sc);

    /* ── OS CONCEPT: Mutex lock badge pinned to intersection ── */
    if (fmodf(t, 7.0f) < 0.08f) {
        const char *concept = (inter == 0) ? "pthread_mutex_lock(&mutex_f10)" : "pthread_mutex_lock(&mutex_f11)";
        os_label_add((float)(cx-55), (float)(cy-ROAD_HW-80), "Mutex Lock", concept, (Color){200,175,45,255});
    }

    /* ── OS CONCEPT: Condition variable badge (periodic) ── */
    if (fmodf(t + 3.5f, 7.0f) < 0.08f) {
        const char *concept2 = is_green ? "pthread_cond_broadcast()" : "pthread_cond_wait()";
        os_label_add((float)(cx+20), (float)(cy-ROAD_HW-80), "CondVar", concept2, (Color){255,185,45,255});
    }

    /* ── Crossing counter overlay ── */
    int cross_cnt = (inter==INTERSECTION_F10) ? f10_crossing_count : f11_crossing_count;
    if (cross_cnt > 0) {
        char cc_str[16]; snprintf(cc_str,20,"crossing:%d",cross_cnt);
        DrawText(cc_str, cx-MeasureText(cc_str,7)/2, cy-5, 7, (Color){100,220,255,200});
    }
}

/*   DRAW PARKING LOT*/
static void draw_parking_lot(int inter, float t) {
    int bx       = (inter == INTERSECTION_F10) ? PK_F10_X : PK_F11_X;
    int by       = PK_F10_Y;
    int *slots   = (inter == INTERSECTION_F10) ? vis_park_slots_f10 : vis_park_slots_f11;
    int occupied = (inter == INTERSECTION_F10) ? vis_park_f10  : vis_park_f11;
    int qcount   = (inter == INTERSECTION_F10) ? vis_parkq_f10 : vis_parkq_f11;
    Color accent = (inter == INTERSECTION_F10) ? (Color){55,145,235,255} : (Color){145,60,235,255};
    Color accent2= (inter == INTERSECTION_F10) ? (Color){30,100,190,200} : (Color){110,40,190,200};

    int lot_w = 218, lot_h = 138;

    /* ── Access lane from intersection to lot ── */
    int ax = (inter == INTERSECTION_F10) ? F10_CX-12 : F11_CX-12;
    int ay_top = DIST_F10_Y + 6;
    int ay_bot = by - 30;
    /* Lane body */
    DrawRectangle(ax, ay_top, 24, ay_bot-ay_top, (Color){22,24,32,240});
    /* Lane edge lines */
    DrawLine(ax,    ay_top, ax,    ay_bot, (Color){55,60,76,200});
    DrawLine(ax+24, ay_top, ax+24, ay_bot, (Color){55,60,76,200});
    /* Lane center dashes */
    for (int dy = ay_top+10; dy < ay_bot; dy += 20)
        DrawRectangle(ax+10, dy, 4, 12, (Color){195,175,45,140});
    /* Entry arrow */
    int arr_y = ay_bot-20;
    DrawLine(ax+12, arr_y, ax+12, arr_y+14, (Color){55,200,80,160});
    DrawLine(ax+7,  arr_y+5, ax+12, arr_y, (Color){55,200,80,160});
    DrawLine(ax+17, arr_y+5, ax+12, arr_y, (Color){55,200,80,160});

    /* ── Lot outer shell ── */
    /* Drop shadow */
    DrawRectangleRounded((Rectangle){bx-4+3, by-30+3, lot_w+2, lot_h+2}, 0.06f, 6, (Color){0,0,0,110});
    /* Lot background */
    DrawRectangleRounded((Rectangle){bx-4, by-30, lot_w+2, lot_h+2}, 0.06f, 6, (Color){16,18,26,245});
    DrawRectangleRoundedLines((Rectangle){bx-4, by-30, lot_w+2, lot_h+2}, 0.06f, 8, accent);

    /* ── Header bar ── */
    DrawRectangleRounded((Rectangle){bx-4, by-30, lot_w+2, 26}, 0.08f, 6, (Color){14,16,24,255});
    DrawLine(bx-4, by-4, bx+lot_w-2, by-4, accent);

    /* "P" icon */
    DrawRectangleRounded((Rectangle){bx-4, by-30, 26, 26}, 0.15f, 6, accent);
    DrawText("P", bx+3, by-27, 16, WHITE);

    /* Lot title — F10 is PRIMARY per spec; F11 is OVERFLOW */
    const char *title = (inter == INTERSECTION_F10)
        ? "F10  PARKING  LOT  [PRIMARY]"
        : "F11  OVERFLOW  LOT";
    DrawText(title, bx+26, by-26, 10, accent);

    /* Semaphore value display */
    char sem_str[40];
    snprintf(sem_str, sizeof(sem_str), "sem_spots: %d/%d", PARKING_SPOTS-occupied, PARKING_SPOTS);
    DrawText(sem_str, bx+lot_w-MeasureText(sem_str,8)-6, by-26, 8, (Color){130,155,195,210});

    /* ── Internal road aisle ── */
    DrawRectangle(bx, by+28, lot_w-8, 10, (Color){20,22,30,240});
    for (int ax2 = bx; ax2 < bx+lot_w-8; ax2 += 16)
        DrawRectangle(ax2, by+31, 10, 4, (Color){195,175,45,120});

    /* ── Parking spot grid ── */
    int spot_w = 38, spot_h = 26, gap = 3;
    for (int s = 0; s < PARKING_SPOTS; s++) {
        int col = s % 5, row = s / 5;
        int sx = bx + col*(spot_w+gap);
        int sy = by + row*(spot_h+gap+2);

        /* Spot background */
        Color spot_bg = slots[s]
            ? (Color){50,16,16,250}
            : (Color){16,40,16,235};
        DrawRectangle(sx, sy, spot_w, spot_h, spot_bg);

        /* Bay markings (parking lines) */
        DrawLine(sx, sy, sx, sy+spot_h, (Color){65,72,88,210});
        DrawLine(sx+spot_w, sy, sx+spot_w, sy+spot_h, (Color){65,72,88,210});
        DrawLine(sx, sy, sx+spot_w, sy, (Color){45,50,65,150});
        /* Number */
        char snum[4]; snprintf(snum,4,"%d",s+1);
        DrawText(snum, sx+2, sy+1, 7, (Color){65,76,96,170});

        if (slots[s]) {
            /* Parked vehicle body */
            float pulse = 0.78f+0.22f*sinf(t*2.2f+s);
            Color vc = veh_color(TYPE_CAR);
            vc.a = (unsigned char)(220*pulse);
            DrawRectangleRounded((Rectangle){sx+3, sy+3, spot_w-6, spot_h-6}, 0.2f, 4, vc);
            /* Windshield */
            DrawRectangle(sx+6, sy+4, spot_w-12, 7, (Color){140,195,230,175});
            /* Wheels */
            DrawCircle(sx+6,       sy+spot_h-5, 3, (Color){28,28,28,220});
            DrawCircle(sx+spot_w-6,sy+spot_h-5, 3, (Color){28,28,28,220});
            DrawCircle(sx+6,       sy+4,         3, (Color){28,28,28,180});
            DrawCircle(sx+spot_w-6,sy+4,         3, (Color){28,28,28,180});
            /* Occupied red dot */
            DrawCircle(sx+spot_w-4, sy+3, 3, (Color){230,45,45,220});
        } else {
            /* Available: green arrow */
            int mx = sx+spot_w/2, my = sy+spot_h/2;
            DrawLine(mx, my-7, mx, my+7, (Color){28,100,28,140});
            DrawLine(mx-4, my+1, mx, my+7, (Color){28,100,28,140});
            DrawLine(mx+4, my+1, mx, my+7, (Color){28,100,28,140});
            DrawCircle(sx+spot_w-4, sy+3, 3, (Color){45,180,60,200});
        }
    }

    /* ── Queue visualization ── */
    int qy = by+2*(spot_h+gap+2)+10;
    DrawRectangle(bx-4, qy-4, lot_w+2, 28, (Color){18,20,32,220});
    DrawLine(bx-4, qy-4, bx+lot_w-2, qy-4, (Color){45,50,68,180});
    DrawText("QUEUE:", bx, qy+2, 8, (Color){145,155,185,220});

    /* sem_queue label */
    char qsem[30];
    snprintf(qsem, sizeof(qsem), "sem_queue: %d/%d", PARKING_QUEUE_MAX-qcount, PARKING_QUEUE_MAX);
    DrawText(qsem, bx+lot_w-MeasureText(qsem,7)-10, qy+3, 7, (Color){130,155,195,200});

    /* Queue vehicle slots */
    for (int i = 0; i < PARKING_QUEUE_MAX; i++) {
        int qsx = bx+52+i*30;
        if (i < qcount) {
            /* Vehicle waiting in queue */
            DrawRectangleRounded((Rectangle){qsx, qy-1, 26, 20}, 0.25f, 4, (Color){200,145,20,230});
            DrawRectangle(qsx+3, qy+1, 20, 7, (Color){190,165,65,200});
            DrawCircle(qsx+5,  qy+18, 3, (Color){30,30,30,210});
            DrawCircle(qsx+21, qy+18, 3, (Color){30,30,30,210});
        } else {
            DrawRectangleRounded((Rectangle){qsx, qy-1, 26, 20}, 0.25f, 4, (Color){24,28,42,200});
            DrawRectangleRoundedLines((Rectangle){qsx, qy-1, 26, 20}, 0.25f, 4, (Color){40,46,64,180});
        }
    }

    /* ── Occupancy bar ── */
    int bar_x = bx-4, bar_y = qy+28, bar_w = lot_w+2;
    DrawRectangle(bar_x, bar_y, bar_w, 10, (Color){22,26,40,220});
    int fill_w = (occupied > 0) ? (int)(bar_w * occupied / (float)PARKING_SPOTS) : 0;
    Color fill_col = (occupied >= PARKING_SPOTS) ? (Color){210,45,35,235}
                   : (occupied > 6)              ? (Color){220,150,20,235}
                                                 : (Color){40,190,65,235};
    DrawRectangle(bar_x, bar_y, fill_w, 10, fill_col);
    /* Bar border */
    DrawRectangleLines(bar_x, bar_y, bar_w, 10, (Color){45,50,68,180});
    /* Occupancy label */
    char occ[28]; snprintf(occ, 28, "%d / %d occupied", occupied, PARKING_SPOTS);
    DrawText(occ, bar_x+bar_w-MeasureText(occ,7)-6, bar_y-10, 7, (Color){145,155,185,200});

    /* ── OS Concept: sem_wait / sem_post indicator ── */
    {
        int ix = bx+lot_w-110, iy = by-30+lot_h+6;
        Color ic = (occupied > 0) ? (Color){255,140,40,200} : (Color){60,200,90,200};
        const char *sem_op = (occupied > 0) ? "sem_wait()" : "sem_post()";
        DrawRectangleRounded((Rectangle){ix, iy, 108, 16}, 0.35f, 4, (Color){10,12,22,210});
        DrawRectangleRoundedLines((Rectangle){ix, iy, 108, 16}, 0.35f, 4, ic);
        DrawText("OS:", ix+4, iy+4, 7, (Color){140,155,195,200});
        DrawText(sem_op, ix+26, iy+4, 7, ic);
    }
}

/* 
   DRAW VEHICLE SPRITE  */
static void draw_vehicle_sprite(VisVehicle *v, float t) {
    if (!v->active || v->state == 3) return;

    float shake_dx = (screen_shake > 0) ? (float)((rand()%3)-1) : 0.0f;
    float shake_dy = (screen_shake > 0) ? (float)((rand()%3)-1) : 0.0f;
    float px = v->x + shake_dx;
    float py = v->y + shake_dy;

    float dx = v->tx - v->x, dy = v->ty - v->y;
    float heading = (dx*dx+dy*dy > 4.0f)
        ? atan2f(dy,dx) * (180.0f/3.14159f)
        : v->angle;

    Color col = veh_color(v->type);
    if (v->flash > 0 && sinf(t*32)>0) col = WHITE;

    float rad = heading * 3.14159f / 180.0f;
    float fw_x = cosf(rad), fw_y = sinf(rad);
    float rt_x = -sinf(rad), rt_y = cosf(rad);

    /* ── Ground shadow (oval) ── */
    DrawEllipse((int)(px+4), (int)(py+5), 14, 5, (Color){0,0,0,80});

    if (v->type == TYPE_AMBULANCE) {
        /* ── Ambulance: white box van body ── */
        /* Body */
        DrawRectanglePro((Rectangle){px, py, 30, 16}, (Vector2){15,8}, heading, col);
        /* Cab */
        DrawRectanglePro((Rectangle){px+fw_x*6, py+fw_y*6, 12, 14}, (Vector2){6,7}, heading,
                         (Color){220,225,230,255});
        /* Windshield */
        DrawRectanglePro((Rectangle){px+fw_x*9, py+fw_y*9, 9, 10}, (Vector2){4,5}, heading,
                         (Color){140,195,235,200});
        /* Red stripe */
        DrawRectanglePro((Rectangle){px, py-1, 30, 3}, (Vector2){15,1}, heading, (Color){235,28,28,200});
        /* Cross symbol on body */
        DrawRectanglePro((Rectangle){px-4, py-1, 8, 2}, (Vector2){0,0}, heading, (Color){255,255,255,240});
        DrawRectanglePro((Rectangle){px-1, py-4, 2, 8}, (Vector2){0,0}, heading, (Color){255,255,255,240});
        /* Wheels (4 corners) */
        for (int w = 0; w < 4; w++) {
            float wfx = fw_x*((w<2)?10:-10) + rt_x*((w%2==0)?7:-7);
            float wfy = fw_y*((w<2)?10:-10) + rt_y*((w%2==0)?7:-7);
            DrawCircle((int)(px+wfx),(int)(py+wfy), 3, (Color){28,28,28,230});
            DrawCircle((int)(px+wfx),(int)(py+wfy), 1, (Color){60,60,60,200});
        }
        /* Siren lights (alternating red/blue) */
        float s1 = 0.5f+0.5f*sinf(t*18.0f);
        float s2 = 0.5f+0.5f*sinf(t*18.0f+3.14f);
        DrawCircle((int)(px+fw_x*8+rt_x*5),(int)(py+fw_y*8+rt_y*5),
                   4, (Color){50,100,255,(unsigned char)(220*s1)});
        DrawCircle((int)(px+fw_x*8-rt_x*5),(int)(py+fw_y*8-rt_y*5),
                   4, (Color){255,40,40,(unsigned char)(220*s2)});
        /* Siren glow rings */
        float r1 = 0.5f+0.5f*sinf(t*15.0f);
        DrawCircleLines((int)px,(int)py, 28, (Color){255,55,55,(unsigned char)(140*r1)});
        DrawCircleLines((int)px,(int)py, 38, (Color){55,105,255,(unsigned char)(90*r1)});
        /* Headlights */
        DrawCircle((int)(px+fw_x*15),(int)(py+fw_y*15-rt_y*4), 3, (Color){255,255,200,180});
        DrawCircle((int)(px+fw_x*15),(int)(py+fw_y*15+rt_y*4), 3, (Color){255,255,200,180});

    } else if (v->type == TYPE_FIRETRUCK) {
        /* ── Fire Truck: red long truck ── */
        DrawRectanglePro((Rectangle){px, py, 34, 16}, (Vector2){17,8}, heading, col);
        /* Cab at front */
        DrawRectanglePro((Rectangle){px+fw_x*8, py+fw_y*8, 12, 14}, (Vector2){6,7}, heading,
                         (Color){200,40,30,255});
        /* Windshield */
        DrawRectanglePro((Rectangle){px+fw_x*11, py+fw_y*11, 8, 9}, (Vector2){4,4}, heading,
                         (Color){140,195,230,185});
        /* Equipment/ladder on body */
        DrawRectanglePro((Rectangle){px-8, py-3, 18, 6}, (Vector2){9,3}, heading, (Color){180,75,20,220});
        /* Siren */
        float s1 = 0.5f+0.5f*sinf(t*15.0f);
        DrawCircle((int)(px+fw_x*9),(int)(py+fw_y*9), 4, (Color){255,65,55,(unsigned char)(210*s1)});
        DrawCircleLines((int)px,(int)py, 26, (Color){255,75,25,(unsigned char)(110*s1)});
        /* Wheels */
        for (int w = 0; w < 4; w++) {
            float wfx = fw_x*((w<2)?12:-12) + rt_x*((w%2==0)?7:-7);
            float wfy = fw_y*((w<2)?12:-12) + rt_y*((w%2==0)?7:-7);
            DrawCircle((int)(px+wfx),(int)(py+wfy), 3, (Color){28,28,28,230});
        }
        DrawCircle((int)(px+fw_x*16),(int)(py+fw_y*16-rt_y*4), 3, (Color){255,255,200,180});
        DrawCircle((int)(px+fw_x*16),(int)(py+fw_y*16+rt_y*4), 3, (Color){255,255,200,180});

    } else if (v->type == TYPE_BUS) {
        /* ── Bus: large yellow/white rectangle ── */
        DrawRectanglePro((Rectangle){px, py, 38, 18}, (Vector2){19,9}, heading, col);
        /* Windows row */
        for (int wi = -3; wi <= 3; wi++) {
            float wfx = fw_x*(wi*5.5f);
            float wfy = fw_y*(wi*5.5f);
            DrawRectanglePro((Rectangle){px+wfx+rt_x*4, py+wfy+rt_y*4, 5, 7},
                              (Vector2){2,3}, heading, (Color){140,195,235,190});
            DrawRectanglePro((Rectangle){px+wfx-rt_x*5, py+wfy-rt_y*5, 5, 7},
                              (Vector2){2,3}, heading, (Color){140,195,235,190});
        }
        /* Bus number/route */
        DrawRectanglePro((Rectangle){px+fw_x*14, py+fw_y*14, 8, 12}, (Vector2){4,6}, heading,
                         (Color){200,200,50,220});
        /* Wheels (6 wheels for bus) */
        for (int w = 0; w < 4; w++) {
            float wfx = fw_x*((w<2)?14:-14) + rt_x*((w%2==0)?8:-8);
            float wfy = fw_y*((w<2)?14:-14) + rt_y*((w%2==0)?8:-8);
            DrawCircle((int)(px+wfx),(int)(py+wfy), 4, (Color){28,28,28,230});
            DrawCircle((int)(px+wfx),(int)(py+wfy), 2, (Color){60,60,60,200});
        }
        DrawCircleLines((int)px,(int)py, 23, (Color){255,200,0,(unsigned char)(55+40*sinf(t*3))});

    } else if (v->type == TYPE_TRACTOR) {
        /* ── Tractor: green with large rear wheels ── */
        DrawRectanglePro((Rectangle){px, py, 26, 16}, (Vector2){13,8}, heading, col);
        /* Engine hood */
        DrawRectanglePro((Rectangle){px+fw_x*6, py+fw_y*6, 10, 10}, (Vector2){5,5}, heading,
                         (Color){100,82,38,230});
        /* Large rear wheels */
        DrawCircle((int)(px-fw_x*10+rt_x*8),(int)(py-fw_y*10+rt_y*8), 7, (Color){35,35,35,230});
        DrawCircle((int)(px-fw_x*10+rt_x*8),(int)(py-fw_y*10+rt_y*8), 4, (Color){65,60,50,210});
        DrawCircle((int)(px-fw_x*10-rt_x*8),(int)(py-fw_y*10-rt_y*8), 7, (Color){35,35,35,230});
        DrawCircle((int)(px-fw_x*10-rt_x*8),(int)(py-fw_y*10-rt_y*8), 4, (Color){65,60,50,210});
        /* Small front wheels */
        DrawCircle((int)(px+fw_x*10+rt_x*5),(int)(py+fw_y*10+rt_y*5), 3, (Color){40,40,40,220});
        DrawCircle((int)(px+fw_x*10-rt_x*5),(int)(py+fw_y*10-rt_y*5), 3, (Color){40,40,40,220});
        /* Exhaust pipe */
        DrawLine((int)(px+fw_x*8+rt_x*2),(int)(py+fw_y*8+rt_y*2),
                 (int)(px+fw_x*8+rt_x*2-rt_x*4),(int)(py+fw_y*8+rt_y*2-rt_y*4),
                 (Color){80,80,80,200});

    } else if (v->type == TYPE_BIKE) {
        /* ── Motorbike: small, sleek ── */
        DrawRectanglePro((Rectangle){px, py, 18, 8}, (Vector2){9,4}, heading, col);
        /* Rider silhouette */
        DrawRectanglePro((Rectangle){px, py-2, 10, 6}, (Vector2){5,3}, heading, (Color){50,50,60,240});
        /* Helmet */
        DrawCircle((int)(px+fw_x*3),(int)(py+fw_y*3), 4, (Color){60,70,90,240});
        /* Wheels */
        DrawCircle((int)(px+fw_x*8),(int)(py+fw_y*8), 4, (Color){32,32,32,230});
        DrawCircle((int)(px+fw_x*8),(int)(py+fw_y*8), 2, (Color){70,70,70,200});
        DrawCircle((int)(px-fw_x*8),(int)(py-fw_y*8), 4, (Color){32,32,32,230});
        DrawCircle((int)(px-fw_x*8),(int)(py-fw_y*8), 2, (Color){70,70,70,200});
        /* Headlight */
        DrawCircle((int)(px+fw_x*9),(int)(py+fw_y*9), 3, (Color){255,255,210,190});
        DrawCircle((int)(px+fw_x*9),(int)(py+fw_y*9), 7, (Color){255,240,180,40});

    } else {
        /* ── Standard Car — best detail ── */
        /* Body (main rectangle) */
        DrawRectanglePro((Rectangle){px, py, 24, 13}, (Vector2){12,6}, heading, col);
        /* Cabin (higher/different color) */
        DrawRectanglePro((Rectangle){px-fw_x*1, py-fw_y*1, 14, 11}, (Vector2){7,5}, heading,
                         (Color){(unsigned char)(col.r*0.85f),(unsigned char)(col.g*0.85f),(unsigned char)(col.b*0.85f),255});
        /* Windshield (front) */
        DrawRectanglePro((Rectangle){px+fw_x*4+rt_x*0.5f, py+fw_y*4+rt_y*0.5f, 9, 7},
                          (Vector2){4,3}, heading, (Color){140,200,235,200});
        /* Rear window */
        DrawRectanglePro((Rectangle){px-fw_x*5-rt_x*0.5f, py-fw_y*5-rt_y*0.5f, 7, 7},
                          (Vector2){3,3}, heading, (Color){130,185,220,160});
        /* 4 wheels */
        for (int w = 0; w < 4; w++) {
            float wfx = fw_x*((w<2)?8:-8) + rt_x*((w%2==0)?5.5f:-5.5f);
            float wfy = fw_y*((w<2)?8:-8) + rt_y*((w%2==0)?5.5f:-5.5f);
            DrawCircle((int)(px+wfx),(int)(py+wfy), 3, (Color){28,28,28,235});
            DrawCircle((int)(px+wfx),(int)(py+wfy), 1, (Color){70,70,70,210});
        }
        /* Headlights */
        DrawCircle((int)(px+fw_x*12-rt_x*3),(int)(py+fw_y*12-rt_y*3), 2, (Color){255,255,200,200});
        DrawCircle((int)(px+fw_x*12+rt_x*3),(int)(py+fw_y*12+rt_y*3), 2, (Color){255,255,200,200});
        /* Headlight glow */
        DrawCircle((int)(px+fw_x*13),(int)(py+fw_y*13), 6, (Color){255,255,180,40});
        /* Tail lights */
        DrawCircle((int)(px-fw_x*12-rt_x*3),(int)(py-fw_y*12-rt_y*3), 2, (Color){255,55,55,180});
        DrawCircle((int)(px-fw_x*12+rt_x*3),(int)(py-fw_y*12+rt_y*3), 2, (Color){255,55,55,180});
    }

    /* ── Vehicle ID badge (HUD overlay) ── */
    char id_str[6]; snprintf(id_str, 6, "V%d", v->id);
    /* Small badge above vehicle */
    int id_w = MeasureText(id_str, 7);
    DrawRectangleRounded((Rectangle){px-id_w/2-3, py-28, id_w+6, 12}, 0.4f, 4,
                         (Color){8,10,20,(unsigned char)(170)});
    DrawText(id_str, (int)(px-id_w/2), (int)(py-27), 7, (Color){200,210,255,210});

    /* ── Progress bar (thin, above badge) ── */
    if (v->progress > 0 && v->state != 3) {
        DrawRectangle((int)px-16, (int)py-33, 32, 4, (Color){14,16,26,210});
        Color bar_col = (v->state==1) ? (Color){50,220,70,225}
                      : (v->state==2) ? (Color){65,140,255,225}
                                      : (Color){255,190,40,225};
        DrawRectangle((int)px-16, (int)py-33, (int)(32*v->progress), 4, bar_col);
        DrawRectangleLines((int)px-16, (int)py-33, 32, 4, (Color){30,36,52,160});
    }

    /* ── State dot (top-right of vehicle) ── */
    Color dot_col = (v->state==0) ? (Color){255,210,0,255}
                  : (v->state==1) ? (Color){50,225,60,255}
                  : (v->state==2) ? (Color){65,140,255,255}
                                  : (Color){70,70,70,80};
    DrawCircle((int)px+16, (int)py-15, 4, dot_col);
    DrawCircle((int)px+16, (int)py-15, 7, (Color){dot_col.r,dot_col.g,dot_col.b,50});

    /* Type label below vehicle */
    DrawText(type_short(v->type), (int)px-8, (int)py+14, 7, (Color){200,210,225,180});
}

/*   DRAW IPC PIPE ANIMATION
*/
static void draw_pipe_animation(float t) {
    int y1  = ROAD_Y - ROAD_HW - 30;
    int y2  = y1 + 16;
    int x1  = F10_CX + ROAD_HW + 16;
    int x2  = F11_CX - ROAD_HW - 16;
    int pw  = x2 - x1;

    /* Pipe tubes */
    DrawRectangle(x1, y1 - 5, pw, 10, (Color){18, 28, 52, 210});
    DrawRectangle(x1, y2 - 5, pw, 10, (Color){28, 16, 48, 210});

    /* Pipe borders */
    DrawLine(x1, y1 - 5, x2, y1 - 5, (Color){55, 100, 200, 190});
    DrawLine(x1, y1 + 5, x2, y1 + 5, (Color){55, 100, 200, 190});
    DrawLine(x1, y2 - 5, x2, y2 - 5, (Color){175, 75, 200, 190});
    DrawLine(x1, y2 + 5, x2, y2 + 5, (Color){175, 75, 200, 190});

    /* End caps */
    DrawRectangle(x1 - 5, y1 - 7, 5, 14, (Color){75, 118, 218, 210});
    DrawRectangle(x2,     y1 - 7, 5, 14, (Color){75, 118, 218, 210});
    DrawRectangle(x1 - 5, y2 - 7, 5, 14, (Color){175, 75, 218, 210});
    DrawRectangle(x2,     y2 - 7, 5, 14, (Color){175, 75, 218, 210});

    /* IPC packets — pipe 1: F10 → F11 */
    for (int i = 0; i < 4; i++) {
        float phase = fmodf(t * 0.6f + i * 0.25f, 1.0f);
        int dx = (int)(x1 + phase * pw);
        unsigned char a = (unsigned char)(140 + 100 * sinf(phase * 3.14f));
        DrawRectangle(dx - 6, y1 - 3, 12, 6, (Color){90, 175, 255, a});
        DrawRectangle(dx - 11, y1 - 2, 5, 4, (Color){55, 115, 200, 55});
    }

    /* IPC packets — pipe 2: F11 → F10 */
    for (int i = 0; i < 4; i++) {
        float phase = fmodf(t * 0.6f + i * 0.25f, 1.0f);
        int dx = (int)(x2 - phase * pw);
        unsigned char a = (unsigned char)(140 + 100 * sinf(phase * 3.14f));
        DrawRectangle(dx - 6, y2 - 3, 12, 6, (Color){195, 95, 255, a});
        DrawRectangle(dx + 6,  y2 - 2,  5,  4, (Color){145, 55, 200, 55});
    }

    /* Labels */
    DrawText("IPC pipe: F10 -> F11", x1 + pw/2 - 55, y1 - 18, 8, (Color){255, 255, 80, 255});
    DrawText("IPC pipe: F11 -> F10", x1 + pw/2 - 55, y2 + 8,  8, (Color){80, 255, 220, 255});

    /* Fork() labels near pipe ends */
    draw_rounded_badge(x1 - 65, y1 - 9, 62, 18,
                       (Color){20, 28, 50, 210}, (Color){60, 100, 200, 180},
                       "fork() F10", 8, (Color){100, 180, 255, 220});
    draw_rounded_badge(x2 + 2, y2 - 9, 62, 18,
                       (Color){30, 18, 48, 210}, (Color){160, 70, 200, 180},
                       "fork() F11", 8, (Color){190, 120, 255, 220});

    /* Floating pipe messages */
    for (int i = 0; i < MAX_PMSG; i++) {
        if (!vis_pmsgs[i].alive) continue;
        PMsg *pm = &vis_pmsgs[i];
        int mx = x1 + (int)(pw * pm->progress);
        if (pm->dir == 1) mx = x2 - (int)(pw * pm->progress);
        int my = (pm->dir == 0) ? y1 - 22 : y2 + 12;
        DrawText(pm->txt, mx, my, 8, (Color){255, 220, 55, (unsigned char)(pm->alpha * 255)});
    }
}

/*    DRAW OS CONCEPT LABELS 
 */
static void draw_os_labels(void) {
    for (int i = 0; i < MAX_OS_LABELS; i++) {
        if (!os_labels[i].active) continue;
        OSLabel *l = &os_labels[i];
        Color bg  = (Color){12, 14, 24, (unsigned char)(200 * l->alpha)};
        Color brd = l->col;
        brd.a = (unsigned char)(220 * l->alpha);
        Color tc  = (Color){255, 255, 255, (unsigned char)(240 * l->alpha)};
        Color sc  = l->col;
        sc.a = (unsigned char)(230 * l->alpha);

        /* Measure both lines to size box properly */
        int tw1 = MeasureText(l->label,   7);
        int tw2 = MeasureText(l->concept, 8);
        int content_w = (tw1 > tw2) ? tw1 : tw2;
        int bw = content_w + 20;   /* 10px padding each side */
        int bh = 30;               /* enough for 2 lines + padding */

        /* Clamp to screen bounds so labels never slide off */
        float rx = l->x - bw / 2.0f;
        if (rx < 2.0f) rx = 2.0f;
        if (rx + bw > SIM_AREA_W - 2) rx = (float)(SIM_AREA_W - bw - 2);

        float ry = l->y;
        if (ry < 54.0f) ry = 54.0f;
        if (ry + bh > SCREEN_H - 54.0f) ry = (float)(SCREEN_H - bh - 54);

        DrawRectangleRounded((Rectangle){rx, ry, (float)bw, (float)bh}, 0.35f, 6, bg);
        DrawRectangleRoundedLines((Rectangle){rx, ry, (float)bw, (float)bh}, 0.35f, 6, brd);
        /* Left colour bar */
        DrawRectangleRounded((Rectangle){rx, ry, 4, (float)bh}, 0.3f, 4, brd);
        DrawText(l->label,   (int)(rx + 8), (int)(ry + 4),  7, sc);
        DrawText(l->concept, (int)(rx + 8), (int)(ry + 15), 8, tc);
    }
}

/* 
   OS CONCEPTS LEGEND PANEL (top-left, always visible)
*/
static void draw_os_legend(float t) {
    /* Narrow panel that stays within screen */
    int lx = 4, ly = 52, lw = 190, lh = 196;
    DrawRectangleRounded((Rectangle){lx, ly, lw, lh}, 0.08f, 8, (Color){10, 12, 22, 228});
    DrawRectangleRoundedLines((Rectangle){lx, ly, lw, lh}, 0.08f, 8, (Color){60, 80, 140, 200});
    DrawText("OS CONCEPTS", lx + 8, ly + 5, 9, (Color){140, 165, 220, 230});
    DrawLine(lx + 4, ly + 17, lx + lw - 4, ly + 17, (Color){55, 70, 120, 160});

    struct { const char *name; const char *desc; Color col; } concepts[] = {
        {"fork()",       "2 ctrl processes",  (Color){120, 210, 255, 255}},
        {"pthread",      "15 veh threads",    (Color){100, 235, 120, 255}},
        {"mutex_lock",   "Intersect lock",    (Color){255, 210, 60, 255}},
        {"cond_wait",    "Wait GREEN sig",    (Color){255, 170, 60, 255}},
        {"sem_wait",     "Parking slots",     (Color){180, 130, 255, 255}},
        {"pipe/write",   "Bidir IPC pipes",   (Color){90, 170, 255, 255}},
        {"SIGINT",       "Graceful shutdown", (Color){255, 100, 100, 255}},
        {"sem_destroy",  "Cleanup rsrcs",     (Color){100, 225, 200, 255}},
    };

    for (int i = 0; i < 8; i++) {
        int cy2 = ly + 22 + i * 22;
        float pulse = 0.7f + 0.3f * sinf(t * 1.8f + i * 0.7f);
        Color dot = concepts[i].col;
        dot.a = (unsigned char)(200 * pulse);
        DrawCircle(lx + 11, cy2 + 6, 4, dot);
        DrawText(concepts[i].name, lx + 20, cy2,      8, concepts[i].col);
        DrawText(concepts[i].desc, lx + 20, cy2 + 11, 7, (Color){140, 150, 175, 175});
    }
}

/*    RIGHT PANEL

 */
static void draw_right_panel(float t) {
    int x = RP_X, w = RIGHT_PANEL_W - 14, h = SCREEN_H - 18;

    DrawRectangleRounded((Rectangle){x, 8, w, h}, 0.06f, 10, (Color){13, 15, 26, 248});
    DrawRectangleRoundedLines((Rectangle){x, 8, w, h}, 0.06f, 10, (Color){55, 68, 92, 210});

    /* Tabs — 5 tabs sized to fit within panel width */
    int tx2 = x + 4, ty2 = 16, tw2 = 66, th2 = 26;
    const char *tabs[] = {"INFO", "STATS", "VEHS", "SEMS", "LOG"};
    Color tab_icons[] = {
        (Color){100, 180, 255, 255},
        (Color){100, 230, 120, 255},
        (Color){255, 200, 80, 255},
        (Color){180, 130, 255, 255},
        (Color){200, 200, 200, 255}
    };
    for (int i = 0; i < 5; i++) {
        Color tc = (active_tab == i) ? tab_icons[i] : (Color){60, 70, 95, 210};
        DrawRectangleRounded((Rectangle){tx2 + i*(tw2+2), ty2, tw2, th2}, 0.25f, 6, tc);
        Color ltc = (active_tab == i) ? (Color){10, 10, 20, 255} : (Color){140, 155, 185, 200};
        DrawText(tabs[i], tx2 + i*(tw2+2) + tw2/2 - MeasureText(tabs[i], 9)/2, ty2 + 8, 9, ltc);
    }

    int cy = ty2 + th2 + 10;
    DrawLine(x + 4, cy, x + w - 4, cy, (Color){55, 68, 92, 150});
    cy += 8;

    /* TAB 0: INFO */
    if (active_tab == 0) {
        DrawText("DUAL INTERSECTION TRAFFIC SIM", x + 10, cy, 12, (Color){160, 200, 255, 255}); cy += 18;
        DrawText("F10 & F11 | Operating System Project", x + 10, cy, 8, (Color){110, 130, 175, 195}); cy += 16;
        DrawLine(x + 4, cy, x + w - 4, cy, (Color){55, 68, 92, 120}); cy += 10;

        struct { const char *lbl; const char *val; Color c; } info[] = {
            {"Vehicles",     "15 pthreads",         (Color){100, 200, 255, 255}},
            {"Parking",      "10 spots/lot",         (Color){100, 255, 110, 255}},
            {"Queue",        "5 slots/lot",          (Color){255, 200, 80,  255}},
            {"IPC",          "Bidir pipes",          (Color){200, 150, 255, 255}},
            {"Priority",     "EMG > BUS > NORM",     (Color){255, 120, 80,  255}},
            {"Controllers",  "2x fork()",            (Color){200, 200, 100, 255}},
            {"Synchro",      "Mutex + CondVar",      (Color){120, 220, 200, 255}},
            {"Signals",      "SIGINT handled",       (Color){255, 100, 100, 255}},
        };
        for (int i = 0; i < 8; i++) {
            DrawRectangle(x + 10, cy, 4, 10, info[i].c);
            DrawText(info[i].lbl, x + 20, cy,     8, LIGHTGRAY);
            DrawText(info[i].val, x + 120, cy,    9, info[i].c);
            cy += 15;
        }
        cy += 6;
        DrawLine(x + 4, cy, x + w - 4, cy, (Color){55, 68, 92, 120}); cy += 10;
        DrawText("VEHICLE TYPES", x + 10, cy, 11, LIGHTGRAY); cy += 14;

        for (int i2 = 0; i2 < 6; i2++) {
            Color vc = veh_color(i2);
            DrawRectangleRounded((Rectangle){x + 10, cy, 16, 13}, 0.25f, 4, vc);
            DrawText(type_short(i2), x + 30, cy + 2, 8, WHITE);
            DrawText(type_str(i2),   x + 72, cy + 2, 8, (Color){175, 180, 205, 200});
            int prio = (i2 <= 1) ? PRIORITY_EMERGENCY : (i2 == 2) ? PRIORITY_BUS : PRIORITY_NORMAL;
            const char *plbl = (prio == 1) ? "EMG" : (prio == 2) ? "BUS" : "NRM";
            draw_rounded_badge(x + w - 44, cy - 1, 40, 15, (Color){20, 22, 35, 200},
                               prio_color(prio), plbl, 8, prio_color(prio));
            cy += 20;
        }
    }

    /* TAB 1: STATS */
    else if (active_tab == 1) {
        DrawText("SIMULATION STATISTICS", x + 10, cy, 12, LIGHTGRAY); cy += 18;

        float prog = (float)vis_stat_done / NUM_VEHICLES;
        DrawText("Overall Progress", x + 10, cy, 9, LIGHTGRAY); cy += 11;
        DrawRectangle(x + 10, cy, w - 22, 14, (Color){38, 42, 58, 210});
        DrawRectangle(x + 10, cy, (int)((w - 22) * prog), 14, (Color){65, 195, 80, 225});
        char pstr[24]; snprintf(pstr, sizeof(pstr), "%d / %d done", vis_stat_done, NUM_VEHICLES);
        DrawText(pstr, x + w/2 - MeasureText(pstr, 9)/2, cy + 2, 9, WHITE);
        cy += 22;

        DrawLine(x + 4, cy, x + w - 4, cy, (Color){55, 68, 92, 120}); cy += 10;

        /* F10 */
        DrawRectangle(x + 10, cy, 6, 60, (Color){60, 140, 220, 180});
        DrawText("INTERSECTION F10", x + 22, cy, 11, (Color){90, 170, 255, 255}); cy += 16;
        char buf[24];
        DrawText("Crossed:",     x + 28, cy, 9, LIGHTGRAY);
        snprintf(buf, 24, "%d", vis_stat_crossed[0]); DrawText(buf, x + 120, cy, 9, GREEN); cy += 14;
        DrawText("Parked:",      x + 28, cy, 9, LIGHTGRAY);
        snprintf(buf, 24, "%d", vis_stat_parked[0]);  DrawText(buf, x + 120, cy, 9, (Color){100, 200, 100, 255}); cy += 14;
        DrawText("Emergencies:", x + 28, cy, 9, LIGHTGRAY);
        snprintf(buf, 24, "%d", vis_stat_emg[0]);     DrawText(buf, x + 120, cy, 9, RED); cy += 20;

        /* F11 */
        DrawRectangle(x + 10, cy, 6, 60, (Color){140, 70, 220, 180});
        DrawText("INTERSECTION F11", x + 22, cy, 11, (Color){185, 120, 255, 255}); cy += 16;
        DrawText("Crossed:",     x + 28, cy, 9, LIGHTGRAY);
        snprintf(buf, 24, "%d", vis_stat_crossed[1]); DrawText(buf, x + 120, cy, 9, GREEN); cy += 14;
        DrawText("Parked:",      x + 28, cy, 9, LIGHTGRAY);
        snprintf(buf, 24, "%d", vis_stat_parked[1]);  DrawText(buf, x + 120, cy, 9, (Color){100, 200, 100, 255}); cy += 14;
        DrawText("Emergencies:", x + 28, cy, 9, LIGHTGRAY);
        snprintf(buf, 24, "%d", vis_stat_emg[1]);     DrawText(buf, x + 120, cy, 9, RED);
    }

    /* TAB 2: VEHICLES */
    else if (active_tab == 2) {
        DrawText("ACTIVE VEHICLES", x + 10, cy, 12, LIGHTGRAY); cy += 16;
        pthread_mutex_lock(&vis_mutex);
        int shown = 0;
        for (int i = 0; i < vis_veh_count && cy < SCREEN_H - 55; i++) {
            VisVehicle *v = &vis_vehicles[i];
            if (!v->active) continue;
            shown++;
            Color vc = veh_color(v->type);
            DrawRectangleRounded((Rectangle){x + 10, cy, 16, 14}, 0.2f, 4, vc);
            char vstr[8]; snprintf(vstr, 8, "V%d", v->id);
            DrawText(vstr,         x + 30, cy + 2, 9, WHITE);
            DrawText(type_short(v->type), x + 58, cy + 2, 9, vc);
            const char *state = (v->state == 0) ? "WAITING"
                               :(v->state == 1) ? "CROSSING"
                               :(v->state == 2) ? "PARKING"
                                                : "DONE";
            Color sc = (v->state == 0) ? (Color){255, 215, 0, 255}
                     : (v->state == 1) ? (Color){55, 225, 60, 255}
                     : (v->state == 2) ? (Color){75, 145, 255, 255}
                                       : (Color){100, 100, 100, 200};
            DrawText(state, x + 100, cy + 2, 8, sc);
            char route[24]; snprintf(route, 24, "%s -> %s", v->origin, v->destination);
            DrawText(route, x + w - MeasureText(route, 7) - 8, cy + 3, 7, (Color){140, 148, 175, 175});
            cy += 19;
        }
        if (!shown) DrawText("(no active vehicles)", x + 20, cy, 10, GRAY);
        pthread_mutex_unlock(&vis_mutex);
    }

    /* TAB 3: SEMAPHORES */
    else if (active_tab == 3) {
        DrawText("SEMAPHORE STATUS", x + 10, cy, 12, LIGHTGRAY); cy += 18;

        /* F10 */
        DrawText("F10 sem_spots (capacity 10)", x + 10, cy, 9, (Color){90, 170, 255, 255}); cy += 12;
        for (int i = 0; i < PARKING_SPOTS; i++) {
            Color sc = (i < PARKING_SPOTS - vis_park_f10)
                ? (Color){40, 175, 40, 225}
                : (Color){195, 45, 38, 225};
            DrawRectangleRounded((Rectangle){x + 10 + i*22, cy, 18, 14}, 0.25f, 4, sc);
        }
        cy += 18;
        DrawText("F10 sem_queue (capacity 5)", x + 10, cy, 9, (Color){90, 170, 255, 255}); cy += 12;
        for (int i = 0; i < PARKING_QUEUE_MAX; i++) {
            Color sc = (i < PARKING_QUEUE_MAX - vis_parkq_f10)
                ? (Color){40, 175, 40, 225}
                : (Color){225, 175, 0, 225};
            DrawRectangleRounded((Rectangle){x + 10 + i*32, cy, 28, 14}, 0.25f, 4, sc);
            DrawText(i < PARKING_QUEUE_MAX - vis_parkq_f10 ? "FREE" : "USED",
                     x + 13 + i*32, cy + 3, 7, WHITE);
        }
        cy += 26;

        /* F11 */
        DrawText("F11 sem_spots (capacity 10)", x + 10, cy, 9, (Color){185, 120, 255, 255}); cy += 12;
        for (int i = 0; i < PARKING_SPOTS; i++) {
            Color sc = (i < PARKING_SPOTS - vis_park_f11)
                ? (Color){40, 175, 40, 225}
                : (Color){195, 45, 38, 225};
            DrawRectangleRounded((Rectangle){x + 10 + i*22, cy, 18, 14}, 0.25f, 4, sc);
        }
        cy += 18;
        DrawText("F11 sem_queue (capacity 5)", x + 10, cy, 9, (Color){185, 120, 255, 255}); cy += 12;
        for (int i = 0; i < PARKING_QUEUE_MAX; i++) {
            Color sc = (i < PARKING_QUEUE_MAX - vis_parkq_f11)
                ? (Color){40, 175, 40, 225}
                : (Color){225, 175, 0, 225};
            DrawRectangleRounded((Rectangle){x + 10 + i*32, cy, 28, 14}, 0.25f, 4, sc);
            DrawText(i < PARKING_QUEUE_MAX - vis_parkq_f11 ? "FREE" : "USED",
                     x + 13 + i*32, cy + 3, 7, WHITE);
        }
        cy += 26;

        DrawText("Green=available  Red=occupied  Yellow=queued",
                 x + 10, cy, 7, LIGHTGRAY);
        cy += 18;
        DrawLine(x + 4, cy, x + w - 4, cy, (Color){55, 68, 92, 120}); cy += 10;

        /* Mutex / cond var state */
        DrawText("MUTEX STATUS", x + 10, cy, 10, LIGHTGRAY); cy += 14;
        struct { const char *nm; int *cnt; Color c; } mtxs[] = {
            {"mutex_f10 crossing_cnt", &f10_crossing_count, (Color){90, 170, 255, 255}},
            {"mutex_f11 crossing_cnt", &f11_crossing_count, (Color){185, 120, 255, 255}},
        };
        for (int i = 0; i < 2; i++) {
            DrawText(mtxs[i].nm, x + 12, cy, 8, mtxs[i].c);
            char cs[16]; snprintf(cs, 16, "= %d", *mtxs[i].cnt);
            DrawText(cs, x + w - 28, cy, 9, *mtxs[i].cnt > 0 ? (Color){255, 200, 50, 255} : (Color){50, 200, 80, 255});
            cy += 14;
        }
    }

    /* TAB 4: LOG */
    else if (active_tab == 4) {
        DrawText("EVENT LOG", x + 10, cy, 12, LIGHTGRAY); cy += 18;
        pthread_mutex_lock(&vis_mutex);
        int shown = 0;
        for (int i = vis_log_count - 1; i >= 0 && shown < 38; i--, shown++) {
            int idx = (vis_log_head + i) % MAX_LOG_ENTRIES;
            Color c = vis_log[idx].col;
            c.a = (unsigned char)(200 - shown * 4);
            DrawText(vis_log[idx].msg, x + 10, cy, 8, c);
            cy += 11;
        }
        pthread_mutex_unlock(&vis_mutex);
    }
}

/*    SIGINT */
static void handle_sigint(int sig) {
    (void)sig;
    running = 0;
    write(STDOUT_FILENO, "\n[MAIN] SIGINT received — shutting down...\n", 43);
}

/* SIMULATION ORCHESTRATOR THREAD*/
typedef struct { int spawned; pid_t pid_f10; pid_t pid_f11; } SimArgs;
static SimArgs g_sim_args;

static void *sim_main_thread(void *arg) {
    SimArgs *sa = (SimArgs *)arg;
    pthread_t tids[NUM_VEHICLES];
    int spawned = 0;

    printf("[SIM] Spawning %d vehicle threads...\n\n", NUM_VEHICLES);

    for (int i = 0; i < NUM_VEHICLES && running; i++) {
        Vehicle *v = create_vehicle(i + 1);
        if (pthread_create(&tids[i], NULL, vehicle_thread, v) != 0) {
            perror("pthread_create"); free(v); break;
        }
        spawned++;
        usleep((rand() % (SPAWN_DELAY_MAX - SPAWN_DELAY_MIN) + SPAWN_DELAY_MIN) * 1000);
    }

    for (int i = 0; i < spawned; i++) pthread_join(tids[i], NULL);

    printf(GRN_TXT "\n[SIM] All %d vehicles processed!\n" RST_TXT, spawned);
    sa->spawned = spawned;
    running = 0;

    write(pipe_f10_to_f11[1], MSG_SHUTDOWN, strlen(MSG_SHUTDOWN) + 1);
    write(pipe_f11_to_f10[1], MSG_SHUTDOWN, strlen(MSG_SHUTDOWN) + 1);
    sleep(1);

    kill(sa->pid_f10, SIGTERM);
    kill(sa->pid_f11, SIGTERM);
    waitpid(sa->pid_f10, NULL, 0);
    waitpid(sa->pid_f11, NULL, 0);

    return NULL;
}

/* MAIN */
int main(void) {
    srand((unsigned)time(NULL));

    struct sigaction sa_act;
    memset(&sa_act, 0, sizeof(sa_act));
    sa_act.sa_handler = handle_sigint;
    sigaction(SIGINT, &sa_act, NULL);

    if (pipe(pipe_f10_to_f11) == -1 || pipe(pipe_f11_to_f10) == -1) {
        perror("pipe"); return 1;
    }

    sem_init(&sem_spots_f10, 0, PARKING_SPOTS);
    sem_init(&sem_queue_f10, 0, PARKING_QUEUE_MAX);
    sem_init(&sem_spots_f11, 0, PARKING_SPOTS);
    sem_init(&sem_queue_f11, 0, PARKING_QUEUE_MAX);

    memset(vis_park_slots_f10, 0, sizeof(vis_park_slots_f10));
    memset(vis_park_slots_f11, 0, sizeof(vis_park_slots_f11));
    memset(os_labels, 0, sizeof(os_labels));

    printf(CYN_TXT
         "\n╔══════════════════════════════════════════════════════════════╗\n"
           "║   DUAL INTERSECTION TRAFFIC SIMULATION — PROFESSIONAL v3.0   ║\n"
           "║   F10 & F11 | 15 pthreads | Dual Semaphores | IPC Pipes      ║\n"
           "╚══════════════════════════════════════════════════════════════╝\n\n"
           RST_TXT);

    pid_t pid_f10 = fork();
    if (pid_f10 < 0) { perror("fork F10"); return 1; }
    if (pid_f10 == 0) run_controller_f10();

    pid_t pid_f11 = fork();
    if (pid_f11 < 0) { perror("fork F11"); return 1; }
    if (pid_f11 == 0) run_controller_f11();

    printf("[MAIN] Controllers: F10 PID=%d, F11 PID=%d\n\n", pid_f10, pid_f11);

    g_sim_args.pid_f10 = pid_f10;
    g_sim_args.pid_f11 = pid_f11;
    pthread_t sim_tid;
    pthread_create(&sim_tid, NULL, sim_main_thread, &g_sim_args);

    vis_log_add("Simulation started", YELLOW);
    vis_log_add("Controllers forked: F10 & F11", (Color){140, 150, 200, 255});

    /* ── Raylib Window — fitted for HP EliteBook Folio 9470m (1366×768) ── */
    SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT);
    InitWindow(SCREEN_W, SCREEN_H, "Traffic Simulation — F10 & F11 | OS Project");
    SetWindowPosition((1366 - SCREEN_W) / 2, (768 - SCREEN_H) / 2);
    SetTargetFPS(60);

    while (!WindowShouldClose()) {
        /* Stay open while simulation runs OR while completion screen is held */
        int sim_done = (vis_stat_done >= NUM_VEHICLES);
        if (!running && !sim_done) break;   /* quit early only if not yet complete */
        float t  = (float)GetTime();
        float dt = GetFrameTime();

        /* Sync lights */
        pthread_mutex_lock(&vis_mutex);
        vis_light_f10 = light_f10_green;
        vis_light_f11 = light_f11_green;
        pthread_mutex_unlock(&vis_mutex);

        /* Sync semaphore values */
        int sp10, sp11, sq10, sq11;
        sem_getvalue(&sem_spots_f10, &sp10);
        sem_getvalue(&sem_spots_f11, &sp11);
        sem_getvalue(&sem_queue_f10, &sq10);
        sem_getvalue(&sem_queue_f11, &sq11);
        pthread_mutex_lock(&vis_mutex);
        vis_park_f10  = PARKING_SPOTS - sp10;
        vis_park_f11  = PARKING_SPOTS - sp11;
        vis_parkq_f10 = PARKING_QUEUE_MAX - sq10;
        vis_parkq_f11 = PARKING_QUEUE_MAX - sq11;
        pthread_mutex_unlock(&vis_mutex);

        /* Tab switching */
        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
            Vector2 mp = GetMousePosition();
            int tx2 = RP_X + 4, ty2 = 16, tw2 = 66, th2 = 26;
            for (int i = 0; i < 5; i++) {
                Rectangle r = {tx2 + i*(tw2+2), ty2, tw2, th2};
                if (CheckCollisionPointRec(mp, r)) active_tab = i;
            }
        }
        if (IsKeyPressed(KEY_ONE))   active_tab = 0;
        if (IsKeyPressed(KEY_TWO))   active_tab = 1;
        if (IsKeyPressed(KEY_THREE)) active_tab = 2;
        if (IsKeyPressed(KEY_FOUR))  active_tab = 3;
        if (IsKeyPressed(KEY_FIVE))  active_tab = 4;
        if (IsKeyPressed(KEY_Q) && vis_stat_done < NUM_VEHICLES) { running = 0; break; }

        update_vis(dt);

        BeginDrawing();
        ClearBackground((Color){34, 85, 34, 255});

        /* ── LAYER 0: Green ground fill (grass/park background) ── */
        for (int row = 0; row < DIST_F10_Y; row++) {
            float frac = (float)row / (float)DIST_F10_Y;
            /* Deep green at top lightening slightly toward road */
            DrawLine(0, row, SIM_AREA_W, row,
                     (Color){(unsigned char)(22 + frac*18),
                              (unsigned char)(65 + frac*28),
                              (unsigned char)(22 + frac*12), 255});
        }

        /* ── LAYER 1: City districts (population zones) ── */
        draw_upper_buildings(t);
        draw_city_district(INTERSECTION_F10, t);
        draw_city_district(INTERSECTION_F11, t);

        /* ── LAYER 2: Roads ── */
        draw_roads(t);

        /* ── LAYER 3: Intersections ── */
        draw_intersection(INTERSECTION_F10, t);
        draw_intersection(INTERSECTION_F11, t);

        /* ── LAYER 4: IPC pipe animation ── */
        draw_pipe_animation(t);

        /* ── LAYER 5: Parking lots — F10=PRIMARY (spec), F11=OVERFLOW ── */
        draw_parking_lot(INTERSECTION_F10, t);
        draw_parking_lot(INTERSECTION_F11, t);

        /* ── LAYER 6: Vehicles ── */
        pthread_mutex_lock(&vis_mutex);
        for (int i = 0; i < vis_veh_count; i++)
            draw_vehicle_sprite(&vis_vehicles[i], t);
        pthread_mutex_unlock(&vis_mutex);

        /* ── LAYER 7: OS concept floating labels ── */
        draw_os_labels();

        /* ── LAYER 8: OS legend panel ── */
        draw_os_legend(t);

        /* ── TITLE BAR ── */
        DrawRectangle(0, 0, SIM_AREA_W, 50, (Color){12, 28, 12, 248});
        DrawRectangle(0, 48, SIM_AREA_W, 2, (Color){45, 140, 55, 210});
        DrawText("DUAL INTERSECTION TRAFFIC SIMULATION", 210, 8, 15, (Color){175, 208, 255, 255});
        DrawText("F10 & F11  |  15 pthreads  |  Dual Semaphores  |  Bidir IPC Pipes  |  fork()", 210, 28, 8, (Color){95, 125, 178, 200});

        /* Light pills */
        int pill_x = SIM_AREA_W - 190;
        Color f10p = vis_light_f10 ? (Color){28, 155, 48, 225} : (Color){155, 28, 28, 225};
        Color f11p = vis_light_f11 ? (Color){28, 155, 48, 225} : (Color){155, 28, 28, 225};
        DrawRectangleRounded((Rectangle){pill_x,      10, 88, 18}, 0.5f, 6, f10p);
        DrawRectangleRounded((Rectangle){pill_x + 94, 10, 88, 18}, 0.5f, 6, f11p);
        DrawText(vis_light_f10 ? "F10  GO" : "F10 STOP", pill_x + 10,      14, 9, WHITE);
        DrawText(vis_light_f11 ? "F11  GO" : "F11 STOP", pill_x + 94 + 10, 14, 9, WHITE);

        /* Progress bar */
        float prog_ratio = (float)vis_stat_done / NUM_VEHICLES;
        DrawRectangle(0, 46, SIM_AREA_W, 2, (Color){28, 32, 52, 255});
        DrawRectangle(0, 46, (int)(SIM_AREA_W * prog_ratio), 2, (Color){55, 195, 95, 225});

        /* Right panel divider */
        DrawLine(SIM_AREA_W, 0, SIM_AREA_W, SCREEN_H, (Color){50, 62, 88, 255});
        draw_right_panel(t);

        /* ── EMERGENCY BANNER ── */
        if (vis_emergency) {
            float flash = sinf(t * 12.0f) * 0.5f + 0.5f;
            unsigned char fa = (unsigned char)(175 + 80 * flash);
            DrawRectangle(0, SCREEN_H - 50, SIM_AREA_W, 50, (Color){72, 0, 0, 218});
            DrawRectangle(0, SCREEN_H - 50, SIM_AREA_W, 3, (Color){255, 72, 72, (unsigned char)(200*flash)});
            DrawText("** EMERGENCY VEHICLE ACTIVE  **  ALL INTERSECTIONS CLEARED **",
                     SIM_AREA_W/2 - 245, SCREEN_H - 35, 14, (Color){255, 72, 72, fa});
            /* OS concept label on banner */
            DrawText("IPC: write(pipe_fd, MSG_EMERGENCY...)",
                     SIM_AREA_W/2 - 145, SCREEN_H - 16, 8, (Color){255, 180, 100, (unsigned char)(200*flash)});
        }

        /* ── COMPLETION BANNER ── */
        if (vis_stat_done >= NUM_VEHICLES) {
            /* Record the moment simulation finished (only once) */
            static double completion_time = -1.0;
            if (completion_time < 0.0) completion_time = GetTime();
            double elapsed = GetTime() - completion_time;
            double hold_left = 5.0 - elapsed;  /* 5-second mandatory display */

            DrawRectangle(0, 0, SCREEN_W, SCREEN_H, (Color){0, 0, 0, 215});
            int bx2 = SCREEN_W/2 - 280, by2 = SCREEN_H/2 - 110;
            DrawRectangleRounded((Rectangle){bx2, by2, 560, 230}, 0.08f, 10, (Color){10, 14, 28, 248});
            DrawRectangleRoundedLines((Rectangle){bx2, by2, 560, 230}, 0.08f, 10, (Color){55, 195, 95, 225});
            DrawText("SIMULATION COMPLETE", bx2 + 50, by2 + 20, 28, (Color){55, 225, 95, 255});
            char s1[80], s2[80];
            snprintf(s1, sizeof(s1), "F10:  %d crossed  |  %d parked  |  %d emergencies",
                     vis_stat_crossed[0], vis_stat_parked[0], vis_stat_emg[0]);
            snprintf(s2, sizeof(s2), "F11:  %d crossed  |  %d parked  |  %d emergencies",
                     vis_stat_crossed[1], vis_stat_parked[1], vis_stat_emg[1]);
            DrawText(s1, bx2 + 30, by2 + 80, 13, (Color){155, 200, 255, 255});
            DrawText(s2, bx2 + 30, by2 + 105, 13, (Color){200, 155, 255, 255});
            DrawText("All threads joined | Semaphores destroyed | Pipes closed",
                     bx2 + 50, by2 + 148, 10, (Color){100, 225, 200, 220});

            if (hold_left > 0.0) {
                /* Countdown bar */
                float bar_frac = (float)(hold_left / 5.0);
                DrawRectangle(bx2 + 20, by2 + 168, 520, 10, (Color){30, 35, 55, 220});
                DrawRectangle(bx2 + 20, by2 + 168, (int)(520 * bar_frac), 10, (Color){55, 195, 95, 220});
                DrawRectangleLines(bx2 + 20, by2 + 168, 520, 10, (Color){80, 100, 80, 180});
                char ctdown[48];
                snprintf(ctdown, sizeof(ctdown), "Closing in %.1f s ...", hold_left);
                DrawText(ctdown, bx2 + 195, by2 + 183, 10, (Color){160, 175, 140, 210});
            } else {
                DrawText("Press Q or ESC to exit", bx2 + 165, by2 + 175, 11, (Color){195, 198, 100, 230});
                /* Allow Q/ESC to exit only after 5 s */
                if (IsKeyPressed(KEY_Q) || IsKeyPressed(KEY_ESCAPE)) { running = 0; }
            }
        }

        EndDrawing();
    }

    CloseWindow();

    /* Cleanup */
    running = 0;

    pthread_mutex_lock(&mutex_f10);
    light_f10_green = 1;
    pthread_cond_broadcast(&cond_f10_bus);
    pthread_cond_broadcast(&cond_f10_green);
    pthread_mutex_unlock(&mutex_f10);

    pthread_mutex_lock(&mutex_f11);
    light_f11_green = 1;
    pthread_cond_broadcast(&cond_f11_bus);
    pthread_cond_broadcast(&cond_f11_green);
    pthread_mutex_unlock(&mutex_f11);

    for (int i = 0; i < NUM_VEHICLES; i++) {
        sem_post(&sem_spots_f10); sem_post(&sem_queue_f10);
        sem_post(&sem_spots_f11); sem_post(&sem_queue_f11);
    }

    pthread_join(sim_tid, NULL);

    sem_destroy(&sem_spots_f10);
    sem_destroy(&sem_queue_f10);
    sem_destroy(&sem_spots_f11);
    sem_destroy(&sem_queue_f11);

    close(pipe_f10_to_f11[0]); close(pipe_f10_to_f11[1]);
    close(pipe_f11_to_f10[0]); close(pipe_f11_to_f10[1]);

    pthread_mutex_destroy(&mutex_f10);
    pthread_mutex_destroy(&mutex_f11);
    pthread_mutex_destroy(&mutex_emergency);
    pthread_mutex_destroy(&vis_mutex);
    pthread_cond_destroy(&cond_f10_green);
    pthread_cond_destroy(&cond_f10_bus);
    pthread_cond_destroy(&cond_f11_green);
    pthread_cond_destroy(&cond_f11_bus);

    printf(GRN_TXT
         "\n╔═══════════════════════════════════════════════════════════════╗\n"
           "║  SIMULATION TERMINATED — All resources cleaned up             ║\n"
           "║  F10: %3d crossed | %3d parked | %3d emergencies              ║\n"
           "║  F11: %3d crossed | %3d parked | %3d emergencies              ║\n"
           "╚═══════════════════════════════════════════════════════════════╝\n"
           RST_TXT,
           vis_stat_crossed[0], vis_stat_parked[0], vis_stat_emg[0],
           vis_stat_crossed[1], vis_stat_parked[1], vis_stat_emg[1]);

    return 0;
}
