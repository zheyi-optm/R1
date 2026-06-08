#ifndef __POLICY_H__
#define __POLICY_H__

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define GRID_W 3
#define GRID_H 4
#define ENTRY  2
#define EXIT1  10
#define EXIT2  12
#define MAX_K1 3
#define MAX_K2 4
#define MAX_STATES ((GRID_W * GRID_H + 1) * (1 << MAX_K2))  /* ~208 */


extern int kfs1[3];
extern int kfs2[4];
extern int kf;
extern int path[20];                // 存储路径序列（格子编号）
extern int path_len;                // 路径长度
extern int picked_k2[4];            // 被拾取的K2位置
extern int picked_cnt;              // 拾取的K2数量
extern int removed_k1;              // 被移除的K1位置
extern int target_k2;               // 目标拾取K2数量


extern bool policy_solve_best(int *out_path, int *out_path_len,
                       int *out_picked, int *out_picked_cnt,
                       int *out_removed_k1, int *out_target_k2);
extern void policy_init(const int k1[3], const int k2[4], int kf);
                       
#endif

											 
											 