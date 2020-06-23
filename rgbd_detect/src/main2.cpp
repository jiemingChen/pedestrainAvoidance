//
// Created by jieming on 11.02.20.
//

#include <iostream>
#include <cstring>
#include <cstdio>


#include <climits>
using namespace std;
const int MAXN = 305;
const int INF = 0x3f3f3f3f;

unsigned int edges[MAXN][MAXN];   // 记录每个妹子和每个男生的好感度
unsigned int meas[MAXN];      // 每个妹子的期望值
unsigned int pred[MAXN];       // 每个男生的期望值
bool vis_girl[MAXN];    // 记录每一轮匹配匹配过的女生
bool vis_pred[MAXN];     // 记录每一轮匹配匹配过的男生
int match[MAXN];        // 记录每个男生匹配到的妹子 如果没有则为-1
unsigned int slack[MAXN];        // 记录每个汉子如果能被妹子倾心最少还需要多少期望值

int N;


bool dfs(int girl)
{
    vis_girl[girl] = true;

    for (int boy = 0; boy < N; ++boy) {

        if (vis_pred[boy]) continue; // 每一轮匹配 每个男生只尝试一次

        unsigned int gap = meas[girl] + pred[boy] - edges[girl][boy];

        if (gap == 0) {  // 如果符合要求
            vis_pred[boy] = true;
            if (match[boy] == -1 || dfs( match[boy] )) {    // 找到一个没有匹配的男生 或者该男生的妹子可以找到其他人
                match[boy] = girl;
                return true;
            }
        } else {
            slack[boy] = min(slack[boy], gap);  // slack 可以理解为该男生要得到女生的倾心 还需多少期望值 取最小值 备胎的样子【捂脸
        }
    }

    return false;
}

int KM()
{
      // 初始每个男生都没有匹配的女生
     // 初始每个男生的期望值为0
    std::fill(pred, pred + N, 0);
    std::fill(match, match+N, -1);
    // 每个女生的初始期望值是与她相连的男生最大的好感度
    for (int i = 0; i < N; ++i) {
        meas[i] = edges[i][0];
        for (int j = 1; j < N; ++j) {
            meas[i] = max(meas[i], edges[i][j]);
        }
    }

    // 尝试为每一个女生解决归宿问题
    for (int i = 0; i < N; ++i) {

        std::fill(slack, slack + N, UINT_MAX);    // 因为要取最小值 初始化为无穷大

        while (1) {
            std::fill(vis_girl, vis_girl + N, false);
            std::fill(vis_pred, vis_pred + N, false);

            if (dfs(i)) break;  // 找到归宿 退出

            // 如果不能找到 就降低期望值
            // 最小可降低的期望值
            unsigned int d = UINT_MAX;
            for (int j = 0; j < N; ++j)
                if (!vis_pred[j]) d = min(d, slack[j]);

            for (int j = 0; j < N; ++j) {
                // 所有访问过的女生降低期望值
                if (vis_girl[j])
                    meas[j] -= d;
                // 所有访问过的男生增加期望值
                if (vis_pred[j])
                    pred[j] += d;
                    // 没有访问过的boy 因为girl们的期望值降低，距离得到女生倾心又进了一步！
                else
                    slack[j] -= d;
            }
        }
    }

    // 匹配完成 求出所有配对的好感度的和
    int res = 0;
    for (int i = 0; i < N; ++i)
//        res += edges[ match[i] ][i];
        cout<< i<< match[i]<<endl;
    return res;
}

int main()
{
    while (~scanf("%d", &N)) {

        for (int i = 0; i < N; ++i)
            for (int j = 0; j < N; ++j)
                scanf("%d", &edges[i][j]);
        KM();
//        printf("%d\n", KM());
    }
    return 0;
}