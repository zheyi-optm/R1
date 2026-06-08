#include "policy.h"

/* ================================================================
   Grid: 3x4, cells numbered 1..12
        col1 col2 col3
   row4  10   11   12   (y=4)
   row3   7    8    9   (y=3)
   row2   4    5    6   (y=2)
   row1   1    2    3   (y=1)   entry=2
   exits = {10, 12}
   ================================================================ */

int kfs1[3];
int kfs2[4];
int kf;
int path[20];                // 存储路径序列（格子编号）
int path_len;                // 路径长度
int picked_k2[4];            // 被拾取的K2位置
int picked_cnt;              // 拾取的K2数量
int removed_k1;              // 被移除的K1位置
int target_k2;               // 目标拾取K2数量

/* ---- coordinates ---- */
static int cell_x(int n) { return (n - 1) % GRID_W + 1; }
static int cell_y(int n) { return (n - 1) / GRID_W + 1; }
static int cell_num(int x, int y) { return (y - 1) * GRID_W + x; }

/* ---- problem data ---- */
static int g_k1[MAX_K1];   //
static int g_k2[MAX_K2];
static int g_kf;
static int g_blocked[13];

/* ---- BFS queue (static, no heap) ---- */
typedef struct {
    int8_t  x, y;
    uint8_t mask;    /* bit i set = K2[i] picked */
} State;

static State   bfs_queue[MAX_STATES];
static int16_t bfs_parent[MAX_STATES];
static uint8_t bfs_visited[GRID_W + 2][GRID_H + 2][1 << MAX_K2];
static int     bfs_head, bfs_tail;
static const int8_t dx[4] = {-1, 1, 0, 0};
static const int8_t dy[4] = { 0, 0,-1, 1};

/* ---- output ---- */
static int g_path[20];
static int g_path_len;
static int g_picked_k2[4];
static int g_picked_cnt;

/* ================================================================
   init
   ================================================================ */
void policy_init(const int k1[3], const int k2[4], int kf)   //
{
    memcpy(g_k1, k1, sizeof(g_k1));
    memcpy(g_k2, k2, sizeof(g_k2));
    g_kf = kf;
}

/* ================================================================
   popcount fallback for platforms without __builtin_popcount
   ================================================================ */
static int popcnt(uint8_t x)
{
#ifdef __GNUC__
    return __builtin_popcount(x);
#else
    x = x - ((x >> 1) & 0x55);
    x = (x & 0x33) + ((x >> 2) & 0x33);
    return (x + (x >> 4)) & 0x0F;
#endif
}

/* ================================================================
   bfs: shortest path for a given K1 removal + target K2 count
   removed_k1_idx: which K1 to remove (0..2)
   target_k2: how many K2 to pick (1..4)
   returns true if path found, result in g_path / g_picked_k2
   ================================================================ */
static bool bfs_run(int removed_k1_idx, int target_k2)
{
    memset(bfs_visited, 0, sizeof(bfs_visited));

    /* blocked set: remaining K1 + Kf */
    memset(g_blocked, 0, sizeof(g_blocked));
    for (int i = 0; i < MAX_K1; i++)
        if (i != removed_k1_idx) g_blocked[g_k1[i]] = 1;
    g_blocked[g_kf] = 1;

    /* K2 index lookup: 0 = not K2, 1..4 = index+1 */
    int k2_idx[13] = {0};
    for (int i = 0; i < MAX_K2; i++)
        k2_idx[g_k2[i]] = i + 1;

    /* init BFS */
    bfs_head = bfs_tail = 0;
    bfs_queue[0].x = -1; bfs_queue[0].y = -1; bfs_queue[0].mask = 0;
    bfs_parent[0] = -1;
    bfs_visited[0][0][0] = 1;  /* (-1,-1) -> visited[0][0] */
    bfs_tail = 1;

    int entry_x = cell_x(ENTRY), entry_y = cell_y(ENTRY);

    while (bfs_head < bfs_tail) {
        State s = bfs_queue[bfs_head];
        int8_t x = s.x, y = s.y;
        uint8_t mask = s.mask;
        int pc = popcnt(mask);

        /* ---- outside map: must enter ---- */
        if (x == -1) {
            int nx = entry_x, ny = entry_y;
            int n = cell_num(nx, ny);
            if (g_blocked[n]) { bfs_head++; continue; }

            uint8_t nm = mask;
            int ki = k2_idx[n];
            if (ki && !(mask & (1 << (ki - 1)))) {
                nm |= (1 << (ki - 1));
                if (popcnt(nm) > target_k2) { bfs_head++; continue; }
            }
            if (!bfs_visited[nx][ny][nm]) {
                bfs_visited[nx][ny][nm] = 1;
                bfs_queue[bfs_tail] = (State){nx, ny, nm};
                bfs_parent[bfs_tail] = bfs_head;
                bfs_tail++;
            }
            bfs_head++;
            continue;
        }

        /* ---- check exit ---- */
        int cn = cell_num(x, y);
        if ((cn == EXIT1 || cn == EXIT2) && pc == target_k2) {
            g_path_len = 0;
            g_picked_cnt = 0;
            uint8_t fmask = mask;
            int idx = bfs_head;
            while (idx >= 0) {
                if (bfs_queue[idx].x != -1)
                    g_path[g_path_len++] = cell_num(bfs_queue[idx].x, bfs_queue[idx].y);
                idx = bfs_parent[idx];
            }
            for (int i = 0; i < g_path_len / 2; i++) {
                int t = g_path[i];
                g_path[i] = g_path[g_path_len - 1 - i];
                g_path[g_path_len - 1 - i] = t;
            }
            for (int i = 0; i < MAX_K2; i++)
                if (fmask & (1 << i)) g_picked_k2[g_picked_cnt++] = g_k2[i];
            return true;
        }

        /* ---- find adjacent unpicked K2 ---- */
        int adj[4], adj_n = 0;
        for (int d = 0; d < 4; d++) {
            int nx = x + dx[d], ny = y + dy[d];
            if (nx < 1 || nx > GRID_W || ny < 1 || ny > GRID_H) continue;
            int n = cell_num(nx, ny);
            int ki = k2_idx[n];
            if (ki && !(mask & (1 << (ki - 1))))
                adj[adj_n++] = ki - 1;
        }

        /* ---- try all subsets of adjacent K2 ---- */
        for (int sub = 0; sub < (1 << adj_n); sub++) {
            int bits = 0, new_pc = pc;
            for (int i = 0; i < adj_n; i++) {
                if (sub & (1 << i)) { bits |= (1 << adj[i]); new_pc++; }
            }
            if (new_pc > target_k2) continue;
            uint8_t nm = mask | bits;

            if (bits == 0) {
                /* move */
                for (int d = 0; d < 4; d++) {
                    int nx = x + dx[d], ny = y + dy[d];
                    if (nx < 1 || nx > GRID_W || ny < 1 || ny > GRID_H) continue;
                    int n = cell_num(nx, ny);
                    if (g_blocked[n]) continue;
                    int ki = k2_idx[n];
                    if (ki && !(nm & (1 << (ki - 1)))) continue;
                    if (!bfs_visited[nx][ny][nm]) {
                        bfs_visited[nx][ny][nm] = 1;
                        bfs_queue[bfs_tail] = (State){nx, ny, nm};
                        bfs_parent[bfs_tail] = bfs_head;
                        bfs_tail++;
                    }
                }
            } else {
                /* pick K2, stay */
                if (!bfs_visited[x][y][nm]) {
                    bfs_visited[x][y][nm] = 1;
                    bfs_queue[bfs_tail] = (State){x, y, nm};
                    bfs_parent[bfs_tail] = bfs_head;
                    bfs_tail++;
                }
            }
        }
        bfs_head++;
    }
    return false;
}

/* ================================================================
   compact_path: remove consecutive duplicate cells from path
   (caused by picking K2 without moving)
   ================================================================ */
static int compact_path(int *path, int len)
{
    if (len <= 1) return len;
    int w = 1;
    for (int r = 1; r < len; r++) {
        if (path[r] != path[r - 1])
            path[w++] = path[r];
    }
    return w;
}

/* ================================================================
   solve: remove 1 K1, pick target_k2 (2 or 3), find shortest
   ================================================================ */
bool policy_solve(int target_k2,
                  int *out_path, int *out_path_len,
                  int *out_picked, int *out_picked_cnt,
                  int *out_removed_k1)
{
    int best_r = -1, best_len = 999;
    for (int r = 0; r < MAX_K1; r++) {
        if (bfs_run(r, target_k2) && g_path_len < best_len) {
            best_len = g_path_len;
            best_r = r;
            *out_path_len = g_path_len;
            memcpy(out_path, g_path, g_path_len * sizeof(int));
            *out_picked_cnt = g_picked_cnt;
            memcpy(out_picked, g_picked_k2, g_picked_cnt * sizeof(int));
            *out_removed_k1 = g_k1[r];
            *out_path_len = compact_path(out_path, *out_path_len);
        }
    }
    return best_r >= 0;
}

/* ================================================================
   solve_best: try target_k2 = 2, 3, pick shortest path
   ================================================================ */
bool policy_solve_best(int *out_path, int *out_path_len,
                       int *out_picked, int *out_picked_cnt,
                       int *out_removed_k1, int *out_target_k2)     //
{
    int best_len = 999;
    bool found = false;
    for (int tk = 2; tk <= 3; tk++) {
        int path[20], pl, pk[4], pc, rk;
        if (policy_solve(tk, path, &pl, pk, &pc, &rk) && pl < best_len) {
            best_len = pl; found = true;
            *out_path_len = pl; memcpy(out_path, path, pl * sizeof(int));
            *out_picked_cnt = pc; memcpy(out_picked, pk, pc * sizeof(int));
            *out_removed_k1 = rk; *out_target_k2 = tk;
        }
    }
    return found;
}

/* ================================================================
   demo
   compile: gcc -DTEST_MAIN -o policy_test policy.c && ./policy_test
   ================================================================ */
#ifdef TEST_MAIN
int main(void)
{
    int k1[3] = {3, 6, 10};
    int k2[4] = {2, 4, 8, 11};
    int kf = 9;
    policy_init(k1, k2, kf);

    printf("Map: K1=[%d,%d,%d] K2=[%d,%d,%d,%d] Kf=%d\n",
           k1[0], k1[1], k1[2], k2[0], k2[1], k2[2], k2[3], kf);

    /* pick 2 K2 */
    {
        int path[20], pl, pk[4], pc, rk;
        if (policy_solve(2, path, &pl, pk, &pc, &rk)) {
            printf("\n-- pick 2 K2 --\n  remove K1: %d\n  path: ", rk);
            for (int i = 0; i < pl; i++) printf("%d ", path[i]);
            printf("\n  picked K2: ");
            for (int i = 0; i < pc; i++) printf("%d ", pk[i]);
            printf("  steps: %d\n", pl - 1);
        }
    }

    /* pick 3 K2 */
    {
        int path[20], pl, pk[4], pc, rk;
        if (policy_solve(3, path, &pl, pk, &pc, &rk)) {
            printf("\n-- pick 3 K2 --\n  remove K1: %d\n  path: ", rk);
            for (int i = 0; i < pl; i++) printf("%d ", path[i]);
            printf("\n  picked K2: ");
            for (int i = 0; i < pc; i++) printf("%d ", pk[i]);
            printf("  steps: %d\n", pl - 1);
        }
    }

    /* best */
    {
        int path[20], pl, pk[4], pc, rk, tk;
        if (policy_solve_best(path, &pl, pk, &pc, &rk, &tk)) {
            printf("\n-- BEST --\n  pick %d K2, remove K1: %d\n  path: ", tk, rk);
            for (int i = 0; i < pl; i++) printf("%d ", path[i]);
            printf("\n  picked K2: ");
            for (int i = 0; i < pc; i++) printf("%d ", pk[i]);
            printf("  steps: %d\n", pl - 1);
        } else {
            printf("\nNo solution!\n");
        }
    }
    return 0;
}
#endif
