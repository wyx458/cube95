/* ==========================================================================
 *  魔方 3D  /  CUBE95   --  单文件 / 纯 C / Win32 GDI
 *
 *  风格: Windows 95 / 扫雷式灰阶浮雕 UI, 只有魔方六个面保留彩色。
 *  视图: 真 3D 透视立方体, 可拖拽旋转视角, 转层带连续动画。
 *
 *  交互 (精简版)
 *   - 左键拖动 = 转动魔方 (贴纸上拖 = 该层, 空白处拖 = 最近的层)
 *   - 右键拖动 = 旋转视角; 上下方向按习惯反转, 左右保持; 视角不走指令通道
 *   - 窗口固定尺寸, 不可缩放 / 不可最大化
 *   - 底部只有一行: 主视角 打乱 复位 说明书 指令:[输入框] 输入
 *   - 指令通道只负责转动魔方 + reset 复位; 打乱 = 生成随机指令经同一通道播放
 *
 *  核心设计
 *   - 54 张贴纸 = (位置 p, 法向 n, 颜色), 不硬编码置换表。
 *     转动 = 绕坐标轴把一层内的 (p,n) 做 90 度整数旋转再查回槽位。
 *   - 渲染: 每张贴纸是一个 3D 空间四边形, 经 [层动画旋转] -> [视角旋转]
 *     -> [透视投影] -> 背面剔除 -> 画家算法深度排序 -> GDI Polygon。
 *   - 动画技巧: 命令执行时立即更新逻辑颜色, 渲染时把该层的贴纸按
 *     -k*90° -> 0 反向插值, 于是"新的颜色从旧的位置滑到新位置"。
 *   - 无第三方依赖, 仅 kernel32 / user32 / gdi32 / msvcrt, 静态链接。
 *
 *  编译 (MinGW-w64 / w64devkit):
 *    gcc -O2 -Wall -finput-charset=UTF-8 -fexec-charset=UTF-8 -static -mwindows ^
 *        rubiks.c -o rubiks.exe -lm -lgdi32 -luser32
 *  自检:  gcc -O2 -DSELFTEST rubiks.c -o rubiks_test.exe -lm
 * ========================================================================== */

#define UNICODE
#define _UNICODE
#define WIN32_LEAN_AND_MEAN
#define WINVER        0x0600
#define _WIN32_WINNT  0x0600

#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <wchar.h>
#include <math.h>
#include <time.h>

#ifndef GET_X_LPARAM
#define GET_X_LPARAM(lp) ((int)(short)LOWORD(lp))
#define GET_Y_LPARAM(lp) ((int)(short)HIWORD(lp))
#endif
#ifndef CLEARTYPE_QUALITY
#define CLEARTYPE_QUALITY 5
#endif
#ifndef EM_SETCUEBANNER
#define EM_SETCUEBANNER (WM_USER+1)      /* ECM_FIRST + 1 = 0x1501 */
#endif
#ifndef PI
#define PI 3.14159265358979323846
#endif
#define CAM_D 8.0

/* ==========================================================================
 *  第一部分: 纯逻辑 + 3D 数学 (与界面无关, 可独立自检)
 * ========================================================================== */

typedef struct { int x, y, z; } V3;                 /* 整数网格向量 */
typedef struct { double x, y, z; } P3;              /* 浮点空间向量 */
typedef struct { V3 p, n; unsigned char color; } Slot;
typedef struct { int axis, layer, k; } Move;        /* 绕 +axis 转 k 个 90 度 */

#define NF    6
#define NS    54
#define MAXH  8192

#define F_U 0
#define F_R 1
#define F_F 2
#define F_D 3
#define F_L 4
#define F_B 5

static const V3 NORM[NF] = {
    { 0, 1, 0}, { 1, 0, 0}, { 0, 0, 1},
    { 0,-1, 0}, {-1, 0, 0}, { 0, 0,-1}
};
/* 每个面的两条切向基向量 (n = v x u), 既用于展开也用于 3D 贴纸几何 */
static const V3 UVEC[NF] = {
    { 1, 0, 0}, { 0, 0,-1}, { 1, 0, 0},
    { 1, 0, 0}, { 0, 0, 1}, {-1, 0, 0}
};
static const V3 VVEC[NF] = {
    { 0, 0, 1}, { 0,-1, 0}, { 0,-1, 0},
    { 0, 0,-1}, { 0,-1, 0}, { 0,-1, 0}
};

static Slot slots[NS];
static int  logic_error;

static int  vc(V3 v, int a)      { return a == 0 ? v.x : (a == 1 ? v.y : v.z); }
static V3   vadd(V3 a, V3 b)     { V3 r = { a.x+b.x, a.y+b.y, a.z+b.z }; return r; }
static V3   vscale(V3 a, int s)  { V3 r = { a.x*s, a.y*s, a.z*s }; return r; }
static V3   vneg(V3 a)           { return vscale(a, -1); }
static V3   vcross(V3 a, V3 b)   {
    V3 r = { a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x };
    return r;
}
static int  veq(V3 a, V3 b)      { return a.x==b.x && a.y==b.y && a.z==b.z; }
static int  vaxis(V3 v)          { int a; for (a=0;a<3;a++) if (vc(v,a)) return a; return -1; }

/* 绕 +axis 转 90 度 (右手系, 整数) */
static V3 rot90(V3 v, int a)
{
    V3 r = v;
    if (a == 0)      { r.y = -v.z; r.z =  v.y; }
    else if (a == 1) { r.x =  v.z; r.z = -v.x; }
    else             { r.x = -v.y; r.y =  v.x; }
    return r;
}
static V3 rot_vec(V3 v, int a, int k)
{
    int t = ((k % 4) + 4) % 4;
    while (t-- > 0) v = rot90(v, a);
    return v;
}

/* 绕坐标轴转任意角度 (浮点, 用于动画与视角) */
static P3 rot_axis(P3 v, int a, double ang)
{
    double c = cos(ang), s = sin(ang);
    P3 r = v;
    if (a == 0)      { r.y = v.y*c - v.z*s; r.z = v.y*s + v.z*c; }
    else if (a == 1) { r.x = v.x*c + v.z*s; r.z = -v.x*s + v.z*c; }
    else             { r.x = v.x*c - v.y*s; r.y = v.x*s + v.y*c; }
    return r;
}
static P3 p3(double x, double y, double z) { P3 r = {x,y,z}; return r; }
static P3 p3v(V3 v)                        { return p3((double)v.x,(double)v.y,(double)v.z); }
static P3 p3add(P3 a, P3 b)                { return p3(a.x+b.x,a.y+b.y,a.z+b.z); }
static P3 p3scale(P3 a, double s)          { return p3(a.x*s,a.y*s,a.z*s); }

/* 视角旋转: 先绕 Y (yaw), 再绕 X (pitch); 相机固定在 +Z 看向原点 */
static P3 cam_rot(P3 v, double yaw, double pitch)
{
    double cy = cos(yaw), sy = sin(yaw), cx = cos(pitch), sx = sin(pitch);
    P3 a = p3(v.x*cy + v.z*sy, v.y, -v.x*sy + v.z*cy);
    return p3(a.x, a.y*cx - a.z*sx, a.y*sx + a.z*cx);
}

/* 视角拖拽映射 (纯函数, GUI 与自检共用)
 *   左右: dx > 0 -> yaw 增大   (保持原有方向, 用户说左右没反)
 *   上下: dy > 0 -> pitch 增大 (与旧版符号相反, 用户说上下反了)
 */
static void view_drag(int dx, int dy, double *yaw, double *pitch)
{
    *yaw   += (double)dx * 0.009;
    *pitch += (double)dy * 0.009;
    if (*pitch >  1.25) *pitch =  1.25;
    if (*pitch < -1.25) *pitch = -1.25;
}

static void init_geometry(void)
{
    int f, r, c;
    for (f = 0; f < NF; f++) for (r = 0; r < 3; r++) for (c = 0; c < 3; c++) {
        int i = f*9 + r*3 + c;
        slots[i].p = vadd(NORM[f], vadd(vscale(UVEC[f], c-1), vscale(VVEC[f], r-1)));
        slots[i].n = NORM[f];
    }
}
static void init_solved(void) { int i; for (i=0;i<NS;i++) slots[i].color = (unsigned char)(i/9); }

static int find_slot(V3 p, V3 n)
{
    int i;
    for (i=0;i<NS;i++) if (veq(slots[i].p,p) && veq(slots[i].n,n)) return i;
    return -1;
}
/* 旋转一层: 只搬颜色, 54 个几何槽位保持不变 */
static void rotate_layer(int axis, int layer, int k)
{
    unsigned char tmp[NS];
    int i, j;
    for (i=0;i<NS;i++) tmp[i] = slots[i].color;
    for (i=0;i<NS;i++) {
        if (vc(slots[i].p, axis) != layer) continue;
        j = find_slot(rot_vec(slots[i].p,axis,k), rot_vec(slots[i].n,axis,k));
        if (j < 0) { logic_error = 1; continue; }
        tmp[j] = slots[i].color;
    }
    for (i=0;i<NS;i++) slots[i].color = tmp[i];
}

static Move face_move(int f, int inverse)
{
    Move m;
    int a = vaxis(NORM[f]);
    m.axis = a; m.layer = vc(NORM[f],a);
    m.k = -m.layer;                       /* 从该面正视, 顺时针 */
    if (inverse) m.k = -m.k;
    return m;
}
#ifdef SELFTEST
static Move move_inverse(Move m) { m.k = -m.k; return m; }
#endif
static int  is_solved(void)
{
    int f,i;
    for (f=0;f<NF;f++) for (i=1;i<9;i++)
        if (slots[f*9+i].color != slots[f*9].color) return 0;
    return 1;
}
static int  counts_ok(void)
{
    int cnt[NF] = {0}, i, f;
    for (i=0;i<NS;i++) { if (slots[i].color >= NF) return 0; cnt[slots[i].color]++; }
    for (f=0;f<NF;f++) if (cnt[f] != 9) return 0;
    return 1;
}
static void move_notation(Move m, WCHAR out[8])
{
    WCHAR ch;
    int cwk;
    if (m.axis==0)      ch = m.layer>0 ? L'R' : (m.layer<0 ? L'L' : L'M');
    else if (m.axis==1) ch = m.layer>0 ? L'U' : (m.layer<0 ? L'D' : L'E');
    else                ch = m.layer>0 ? L'F' : (m.layer<0 ? L'B' : L'S');
    cwk = m.layer >= 0 ? -1 : 1;
    out[0] = ch;
    if (m.k==2 || m.k==-2) { out[1]=L'2'; out[2]=0; }
    else if (m.k==cwk)     { out[1]=0; }
    else                   { out[1]=L'\''; out[2]=0; }
}

/* --------------------------------------------------------------------------
 *  指令串解析 (纯逻辑, 命令行 / 打乱 / 自检共用)
 *    U D L R F B = 六个面, M E S = 三个中间切片
 *    小写 = 反向, 后缀 ' (或 ’) = 反向, 后缀 2 = 转 180°
 *    空白 / 逗号 / 分号 分隔, 也接受 RUR'U' 这种连写
 *    返回值: 解析出的步数; -1 = 非法字符, -2 = 超出 max
 * -------------------------------------------------------------------------- */
#define CMD_CAP 64
static int parse_commands(const WCHAR *s, Move out[], int max)
{
    int n = 0;
    while (*s) {
        WCHAR c = *s++;
        Move m;
        if (c==L' ' || c==L'\t' || c==L',' || c==L';' || c==L'\n' || c==L'\r') continue;
        switch (c) {
        case L'U': case L'u': m = face_move(F_U, c==L'u'); break;
        case L'D': case L'd': m = face_move(F_D, c==L'd'); break;
        case L'L': case L'l': m = face_move(F_L, c==L'l'); break;
        case L'R': case L'r': m = face_move(F_R, c==L'r'); break;
        case L'F': case L'f': m = face_move(F_F, c==L'f'); break;
        case L'B': case L'b': m = face_move(F_B, c==L'b'); break;
        case L'M': case L'm': m.axis=0; m.layer=0; m.k=(c==L'M')?-1: 1; break;
        case L'E': case L'e': m.axis=1; m.layer=0; m.k=(c==L'E')?-1: 1; break;
        case L'S': case L's': m.axis=2; m.layer=0; m.k=(c==L'S')?-1: 1; break;
        default: return -1;
        }
        if (*s==L'\'' || *s==(WCHAR)0x2019 || *s==L'`') { m.k = -m.k; s++; }
        else if (*s==L'2') { m.k = 2; s++; }
        if (n >= max) return -2;
        out[n++] = m;
    }
    return n;
}
/* 唯一的非转动指令: reset (大小写不敏感, 前后可有空白) */
static int is_reset_cmd(const WCHAR *s)
{
    static const WCHAR r[5] = { L'r',L'e',L's',L'e',L't' };
    int i;
    while (*s==L' '||*s==L'\t'||*s==L'\r'||*s==L'\n') s++;
    for (i=0;i<5;i++) {
        WCHAR c = *s++;
        if (c>=L'A' && c<=L'Z') c = (WCHAR)(c+32);
        if (c != r[i]) return 0;
    }
    while (*s==L' '||*s==L'\t'||*s==L'\r'||*s==L'\n') s++;
    return *s==0;
}

/* ------------------------------ 游戏状态 (纯逻辑) */

static Move   hist[MAXH];
static int    hlen, hpos, hist_base;   /* hist_base: 前缀中属于打乱的步数 */
static DWORD  elapsed_ms;
static int    timing;                  /* 0 未开始 / 1 计时中 / 2 停止 */
static DWORD  t_start;

static void clear_game(void)
{
    hlen = hpos = hist_base = 0;
    elapsed_ms = 0; timing = 0;
    init_solved();
}
static void start_clock(void)
{
    if (timing != 1) { t_start = GetTickCount() - (DWORD)elapsed_ms; timing = 1; }
}
static DWORD now_elapsed(void) { return timing==1 ? GetTickCount()-t_start : elapsed_ms; }

static void logic_push(Move m)             /* 普通用户操作 */
{
    if (hpos < hlen) { hlen = hpos; if (hist_base > hpos) hist_base = hpos; }
    if (hlen < MAXH) { hist[hlen++] = m; hpos = hlen; }
}
static void logic_push_base(Move m)        /* 打乱步 */
{
    if (hlen < MAXH) { hist[hlen++] = m; hpos = hlen; }
}
#ifdef SELFTEST
static Move logic_undo(int floor)          /* k==0 表示无可撤销 */
{
    Move z = {0,0,0};
    if (hpos <= floor) return z;
    return move_inverse(hist[--hpos]);
}
static Move logic_redo(void)
{
    Move z = {0,0,0};
    if (hpos >= hlen) return z;
    return hist[hpos++];
}
#endif

#define SCRAM_N 22
static void make_scramble(Move out[SCRAM_N])
{
    int i, last = -1;
    for (i=0;i<SCRAM_N;i++) {
        int a, f, inv;
        do { a = rand()%3; } while (a == last);
        f = (a==0) ? (rand()&1 ? F_R : F_L)
          : (a==1) ? (rand()&1 ? F_U : F_D)
                   : (rand()&1 ? F_F : F_B);
        inv = rand()&1;
        out[i] = face_move(f, inv);
        if (rand()%3 == 0) out[i].k = 2;   /* 随机 180° */
        last = a;
    }
}
static void build_scramble_text(const Move mv[SCRAM_N], WCHAR out[256])
{
    int i; WCHAR nb[8];
    out[0] = 0;
    for (i=0;i<SCRAM_N;i++) {
        move_notation(mv[i], nb);
        if (i) wcscat(out, L" ");
        wcscat(out, nb);
    }
}

/* ==========================================================================
 *  第二部分: Win32 界面 (复古灰阶 + 3D 动画 + 拖拽 + 指令输入)
 * ========================================================================== */
#ifndef SELFTEST

/* -------------------------------- 灰阶主题 */
#define C_BG      RGB(192,192,192)
#define C_WHITE   RGB(255,255,255)
#define C_HI      RGB(223,223,223)
#define C_SHADE   RGB(128,128,128)
#define C_DARK    RGB( 64, 64, 64)
#define C_BLACK   RGB(  0,  0,  0)
#define C_STAGE   RGB(168,168,168)
#define C_CUBE    RGB( 48, 48, 48)
#define C_LED_ON  RGB(224,224,224)
#define C_LED_OFF RGB( 34, 34, 34)

static const COLORREF FACE_COLOR[NF] = {
    RGB(238,240,244),  /* U 白 */
    RGB(214, 48, 40),  /* R 红 */
    RGB( 24,158, 88),  /* F 绿 */
    RGB(246,202, 32),  /* D 黄 */
    RGB(240,120, 24),  /* L 橙 */
    RGB( 34, 96,208)   /* B 蓝 */
};
static int clamp255(int v) { return v<0?0:(v>255?255:v); }

/* -------------------------------- 资源 */
static HWND   g_hwnd, g_hedit;
static WNDPROC g_edit_old;
static HBRUSH g_brBg, g_brWhite, g_brHi, g_brShade, g_brDark, g_brBlack;
static HBRUSH g_brStage, g_brCube, g_brLedOn, g_brLedOff, g_brFace[NF];
static HPEN   g_pnBlack, g_pnThin;
static HFONT  g_fUI, g_fSmall;
static HDC    g_memdc; static HBITMAP g_membmp, g_oldbmp;
static int    g_w, g_h;

/* -------------------------------- 视图 / 动画 / 命令 */
static double g_yaw = -0.62, g_pitch = 0.48;
static double g_tYaw = -0.62, g_tPitch = 0.48;
static double g_cx, g_cy, g_scale;

typedef struct { int on, axis, layer, k; DWORD t0; int dur; } Anim;
static Anim anim;

#define QCAP 64
#define K_MOVE  0
#define K_SCRAM 1
typedef struct { Move m; int kind; } Cmd;
static Cmd  queue[QCAP];
static int  qh, qt;
static int  scram_pending;

typedef struct { int idx; POINT q[4]; double depth; COLORREF col; } Item;
static Item items[NS];
static int  nitems;

/* -------------------------------- 复古控件 */
typedef struct { const WCHAR *label; int id; RECT rc; } Btn;
#define NBTN 5
static Btn btns[NBTN];

#define ID_V0    200                       /* 唯一的视角按钮: 主视角 */
#define ID_SHUF  201
#define ID_RESET 202
#define ID_HELP  203
#define ID_RUN   204
#define ID_EDIT  400                       /* 输入框控件 ID */

#define WIN_W     600                      /* 固定窗口外框尺寸 */
#define WIN_H     560
#define APP_TITLE L"CUBE95  ·  3D 魔方"

static int  press_id;
static int  drag_mode;                      /* 0 无 / 1 左键转层 / 2 右键转视角 */
static int  drag_done, drag_idx;
static POINT drag_pt, drag_last;
static RECT g_stage_rect, g_edit_rc, g_label_rc;
static WCHAR g_msg[256];                   /* 短提示: 显示在标题栏, 4 秒后自动消失 */
static DWORD g_msg_until;

/* 不占用底部任何一行: 反馈只放在窗口标题, 且会自动消失 */
static void set_status(const WCHAR *s)
{
    WCHAR t[256];
    wcsncpy(g_msg, s, 255); g_msg[255] = 0;
    g_msg_until = GetTickCount() + 4000;
    if (!g_hwnd) return;
    if (g_msg[0]) _snwprintf(t, 256, L"%ls   —   %ls", APP_TITLE, g_msg);
    else          wcscpy(t, APP_TITLE);
    t[255] = 0;
    SetWindowTextW(g_hwnd, t);
}

static void state_path(WCHAR out[MAX_PATH])
{
    WCHAR app[MAX_PATH];
    DWORD n = GetEnvironmentVariableW(L"APPDATA", app, MAX_PATH);
    if (!n || n >= MAX_PATH) {
        n = GetEnvironmentVariableW(L"USERPROFILE", app, MAX_PATH);
        if (!n || n >= MAX_PATH) { wcscpy(out,L"rubiks_state.bin"); return; }
    }
    _snwprintf(out, MAX_PATH, L"%ls\\RubiksCube", app); out[MAX_PATH-1]=0;
    CreateDirectoryW(out, NULL);
    _snwprintf(out, MAX_PATH, L"%ls\\RubiksCube\\state.bin", app); out[MAX_PATH-1]=0;
}
static void save_state(void)
{
    WCHAR path[MAX_PATH];
    unsigned char cols[NS], hdr[8] = {'R','C','U','B','3',0,0,0};
    int meta[4], i; DWORD wr; HANDLE h;
    state_path(path);
    h = CreateFileW(path, GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
    if (h == INVALID_HANDLE_VALUE) return;
    for (i=0;i<NS;i++) cols[i] = slots[i].color;
    elapsed_ms = now_elapsed();
    meta[0]=hlen; meta[1]=hpos; meta[2]=timing; meta[3]=hist_base;
    WriteFile(h,hdr,8,&wr,NULL);
    WriteFile(h,cols,NS,&wr,NULL);
    WriteFile(h,meta,sizeof(meta),&wr,NULL);
    WriteFile(h,&elapsed_ms,sizeof(elapsed_ms),&wr,NULL);
    WriteFile(h,hist,(DWORD)(sizeof(Move)*(size_t)hlen),&wr,NULL);
    CloseHandle(h);
}
static void load_state(void)
{
    WCHAR path[MAX_PATH];
    unsigned char cols[NS], hdr[8];
    int meta[4], i, bad=0; DWORD rd; HANDLE h;
    state_path(path);
    h = CreateFileW(path, GENERIC_READ, FILE_SHARE_READ, NULL, OPEN_EXISTING,
                    FILE_ATTRIBUTE_NORMAL, NULL);
    if (h == INVALID_HANDLE_VALUE) return;
    if (!ReadFile(h,hdr,8,&rd,NULL) || rd!=8 || memcmp(hdr,"RCUB3",5)) { CloseHandle(h); return; }
    if (!ReadFile(h,cols,NS,&rd,NULL) || rd!=NS) { CloseHandle(h); return; }
    for (i=0;i<NS;i++) if (cols[i] >= NF) bad = 1;
    if (bad) { CloseHandle(h); return; }
    if (!ReadFile(h,meta,sizeof(meta),&rd,NULL) || rd!=sizeof(meta)) { CloseHandle(h); return; }
    if (!ReadFile(h,&elapsed_ms,sizeof(elapsed_ms),&rd,NULL) || rd!=sizeof(elapsed_ms)) { CloseHandle(h); return; }
    if (meta[0]>=0 && meta[0]<=MAXH && meta[1]>=0 && meta[1]<=meta[0]) {
        DWORD need = (DWORD)(sizeof(Move)*(size_t)meta[0]);
        if (meta[0]==0 || (ReadFile(h,hist,need,&rd,NULL) && rd==need)) {
            hlen=meta[0]; hpos=meta[1];
            hist_base = (meta[3]>=0 && meta[3]<=hlen) ? meta[3] : 0;
        }
    }
    CloseHandle(h);
    for (i=0;i<NS;i++) slots[i].color = cols[i];
    timing = (meta[2]==2) ? 2 : 0;        /* 重开后计时暂停 */
}

/* -------------------------------- 命令 / 动画驱动 */
static void enqueue(Cmd c)
{
    if (qt - qh < QCAP) queue[qt++ % QCAP] = c;
}
static void start_anim(Move m, int dur)
{
    anim.on=1; anim.axis=m.axis; anim.layer=m.layer; anim.k=m.k;
    anim.t0=GetTickCount(); anim.dur=dur;
}
static void apply_cmd(Cmd c)
{
    Move m = c.m;
    if (c.kind==K_SCRAM) logic_push_base(m);       /* 打乱步不计入用户步数 */
    else                 { logic_push(m); start_clock(); }
    if (m.k == 0) return;
    rotate_layer(m.axis, m.layer, m.k);
    start_anim(m, c.kind==K_SCRAM ? 24 : 150);
    if (is_solved() && timing==1) {
        elapsed_ms = GetTickCount() - t_start;
        timing = 2;
        set_status(L"六面复原!");
        MessageBeep(MB_OK);
    }
    save_state();
}
static void clear_queue(void)
{
    qh = qt = 0; anim.on = 0; scram_pending = 0;
}
/* 指令通道: as_base=1 表示这是打乱生成的随机指令 */
static void run_commands(const WCHAR *text, int as_base)
{
    Move mv[QCAP];
    int n = parse_commands(text, mv, QCAP-1), i;
    if (n < 0) {
        set_status(n==-2 ? L"指令过多（一次最多 63 条）" : L"无效指令");
        return;
    }
    if (n == 0) { set_status(L"请输入指令"); return; }
    if (as_base) clear_queue(), clear_game();
    else if ((qt-qh) + n > QCAP-1) {
        set_status(L"指令过多，请等待播放结束");
        return;
    }
    for (i=0;i<n;i++) { Cmd c; c.m=mv[i]; c.kind=as_base?K_SCRAM:K_MOVE; enqueue(c); }
    if (as_base) scram_pending = 1;
    else { WCHAR b[64]; _snwprintf(b,64,L"执行 %d 条指令",n); b[63]=0; set_status(b); }
    InvalidateRect(g_hwnd,NULL,FALSE);
}
/* 打乱 = 生成随机指令文本 -> 填入输入框 -> 走同一指令通道播放 */
static void shuffle(void)
{
    Move mv[SCRAM_N]; WCHAR text[256];
    make_scramble(mv);
    build_scramble_text(mv, text);
    SetWindowTextW(g_hedit, text);          /* 打乱序列留在输入框, 可直接复制 */
    run_commands(text, 1);
    set_status(L"已打乱 · 指令在输入框");
    InvalidateRect(g_hwnd,NULL,FALSE);
}
static void do_reset(void)
{
    clear_queue(); clear_game();
    SetWindowTextW(g_hedit, L"");
    set_status(L"已复位");
    save_state();
    InvalidateRect(g_hwnd,NULL,FALSE);
}
static void execute_from_edit(void)
{
    WCHAR text[512];
    if (!g_hedit) return;
    GetWindowTextW(g_hedit, text, 512);
    if (is_reset_cmd(text)) { do_reset(); return; }
    run_commands(text, 0);
    SendMessageW(g_hedit, EM_SETSEL, 0, -1);      /* 全选, 便于直接覆盖输入 */
}
static void do_cmd_id(int id);

/* -------------------------------- 绘制小工具 */
static void fr(HDC dc, int x, int y, int w, int h, HBRUSH b)
{
    RECT r; SetRect(&r,x,y,x+w,y+h); FillRect(dc,&r,b);
}
static void txt(HDC dc, const WCHAR *s, RECT rc, HFONT f, COLORREF col, UINT fmt)
{
    HFONT old = (HFONT)SelectObject(dc,f);
    SetBkMode(dc,TRANSPARENT); SetTextColor(dc,col);
    DrawTextW(dc,s,-1,&rc,fmt);
    SelectObject(dc,old);
}
/* Win95 双层浮雕边框: raised=1 凸起, 0 凹陷 */
static void bevel(HDC dc, int x, int y, int w, int h, int raised)
{
    HBRUSH o1 = raised ? g_brWhite : g_brDark;
    HBRUSH o2 = raised ? g_brDark  : g_brWhite;
    HBRUSH i1 = raised ? g_brHi    : g_brShade;
    HBRUSH i2 = raised ? g_brShade : g_brHi;
    fr(dc,x,y,w,1,o1); fr(dc,x,y,1,h,o1);
    fr(dc,x,y+h-1,w,1,o2); fr(dc,x+w-1,y,1,h,o2);
    fr(dc,x+1,y+1,w-2,1,i1); fr(dc,x+1,y+1,1,h-2,i1);
    fr(dc,x+1,y+h-2,w-2,1,i2); fr(dc,x+w-2,y+1,1,h-2,i2);
}
/* 扫雷式七段数码管, 3 位, 黑底灰字 */
static void led3(HDC dc, int x, int y, int value)
{
    static const unsigned char seg[10] = {
        0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F
    };
    int digits[3], i;
    if (value < 0) value = 0;
    if (value > 999) value = 999;
    digits[0]=value/100; digits[1]=(value/10)%10; digits[2]=value%10;
    fr(dc,x-3,y-3,3*17+4*3+6,36,g_brDark);
    bevel(dc,x-4,y-4,3*17+4*3+8,38,0);
    fr(dc,x,y,3*17+4*3,30,g_brBlack);
    for (i=0;i<3;i++) {
        int bx = x+3+i*(17+3), by = y+3, bits = seg[digits[i]];
        int W = 12, H = 11, t = 3;
        #define SEG(bit, sx, sy, sw, sh) \
            fr(dc, bx+(sx), by+(sy), (sw), (sh), (bits&(bit)) ? g_brLedOn : g_brLedOff)
        SEG(0x01, t,   0,       W-2*t, t);
        SEG(0x02, W-t, t,       t,      H-t);
        SEG(0x04, W-t, H+2,     t,      H-t);
        SEG(0x08, t,   2*H+2,   W-2*t, t);
        SEG(0x10, 0,   H+2,     t,      H-t);
        SEG(0x20, 0,   t,       t,      H-t);
        SEG(0x40, t,   H+1,     W-2*t, t);
        #undef SEG
    }
}

/* -------------------------------- 3D 几何 */
static void quad_at(int f, P3 c, double half, P3 q[4])
{
    P3 u = p3scale(p3v(UVEC[f]), half), v = p3scale(p3v(VVEC[f]), half);
    q[0] = p3add(p3add(c, p3scale(u,-1)), p3scale(v,-1));
    q[1] = p3add(p3add(c, u),              p3scale(v,-1));
    q[2] = p3add(p3add(c, u),              v);
    q[3] = p3add(p3add(c, p3scale(u,-1)),  v);
}
static void sticker_quad(int i, P3 q[4])
{
    V3 p = slots[i].p, n = slots[i].n;
    quad_at(i/9, p3((double)p.x + 0.5*(double)n.x,
                    (double)p.y + 0.5*(double)n.y,
                    (double)p.z + 0.5*(double)n.z), 0.43, q);
}
static void face_quad(int f, P3 q[4])
{
    V3 n = NORM[f];
    quad_at(f, p3(1.5*n.x, 1.5*n.y, 1.5*n.z), 1.5, q);
}
static POINT project(P3 v)
{
    double f = CAM_D / (CAM_D - v.z);
    POINT p;
    p.x = (LONG)(g_cx + v.x*g_scale*f);
    p.y = (LONG)(g_cy - v.y*g_scale*f);
    return p;
}
static int cmp_item(const void *a, const void *b)
{
    double x = ((const Item*)a)->depth, y = ((const Item*)b)->depth;
    return x < y ? -1 : (x > y ? 1 : 0);
}
static COLORREF shade(COLORREF base, P3 n)
{
    double d = 0.62 + 0.38*(0.55*n.y + 0.75*n.z - 0.18*n.x);
    if (d < 0.62) d = 0.62;
    if (d > 1.0) d = 1.0;
    return RGB(clamp255((int)(GetRValue(base)*d)),
               clamp255((int)(GetGValue(base)*d)),
               clamp255((int)(GetBValue(base)*d)));
}
static void draw_poly(HDC dc, POINT q[4], HBRUSH br)
{
    HBRUSH ob = (HBRUSH)SelectObject(dc,br);
    HPEN   op = (HPEN)SelectObject(dc,g_pnBlack);
    Polygon(dc,q,4);
    SelectObject(dc,ob); SelectObject(dc,op);
}
static void draw_cube(HDC dc)
{
    int i, f, a;
    double ang = 0.0;
    P3 bigq[4]; POINT bp[4];

    /* 动画角度: 逻辑已转 k*90, 渲染回退 -k*90*(1-ease) */
    if (anim.on) {
        DWORD now = GetTickCount();
        double t = (double)(now - anim.t0) / (double)anim.dur;
        if (t > 1) t = 1;
        if (t < 0) t = 0;
        ang = -(double)anim.k * (PI/2.0) * (1.0 - t*t*(3.0-2.0*t));
    }

    /* 立方体本体: 三个可见大面 */
    for (f=0;f<NF;f++) {
        P3 n = cam_rot(p3v(NORM[f]), g_yaw, g_pitch);
        if (n.z <= 0.02) continue;
        face_quad(f, bigq);
        for (a=0;a<4;a++) bp[a] = project(cam_rot(bigq[a], g_yaw, g_pitch));
        draw_poly(dc, bp, g_brCube);
    }

    /* 贴纸 */
    nitems = 0;
    for (i=0;i<NS;i++) {
        P3 q[4], n, c;
        int j;
        sticker_quad(i, q);
        n = p3v(NORM[i/9]);
        if (anim.on && vc(slots[i].p, anim.axis) == anim.layer) {
            for (j=0;j<4;j++) q[j] = rot_axis(q[j], anim.axis, ang);
            n = rot_axis(n, anim.axis, ang);
        }
        c = cam_rot(p3((q[0].x+q[1].x+q[2].x+q[3].x)/4.0,
                       (q[0].y+q[1].y+q[2].y+q[3].y)/4.0,
                       (q[0].z+q[1].z+q[2].z+q[3].z)/4.0), g_yaw, g_pitch);
        {
            P3 nr = cam_rot(n, g_yaw, g_pitch);
            if (nr.z <= 0.03) continue;            /* 背面剔除 */
            items[nitems].col = shade(FACE_COLOR[slots[i].color], nr);
        }
        items[nitems].depth = c.z;
        items[nitems].idx = i;
        for (j=0;j<4;j++) items[nitems].q[j] = project(cam_rot(q[j], g_yaw, g_pitch));
        nitems++;
    }
    qsort(items, (size_t)nitems, sizeof(Item), cmp_item);
    for (i=0;i<nitems;i++) {
        HBRUSH br = CreateSolidBrush(items[i].col);
        draw_poly(dc, items[i].q, br);
        DeleteObject(br);
    }
}

/* -------------------------------- 拖拽映射: 屏幕方向 -> 3D 层转动 */
static void screen_dir(P3 center, V3 d, double *sx, double *sy)
{
    P3 a = cam_rot(p3add(center, p3scale(p3v(d),0.5)), g_yaw, g_pitch);
    POINT p0 = project(cam_rot(center, g_yaw, g_pitch));
    POINT p1 = project(a);
    double dx = (double)(p1.x-p0.x), dy = (double)(p1.y-p0.y);
    double L = sqrt(dx*dx+dy*dy);
    if (L < 0.0001) { *sx=0; *sy=0; return; }
    *sx = dx/L; *sy = dy/L;
}
static Move drag_to_move(int idx, int dx, int dy)
{
    int f = idx/9;
    V3 n = NORM[f], u = UVEC[f], v = VVEC[f], d3, ax;
    P3 center = p3((double)slots[idx].p.x + 0.5*(double)n.x,
                   (double)slots[idx].p.y + 0.5*(double)n.y,
                   (double)slots[idx].p.z + 0.5*(double)n.z);
    double sux,suy,svx,svy,du,dv;
    Move m; int a;
    screen_dir(center, u, &sux, &suy);
    screen_dir(center, v, &svx, &svy);
    du = dx*sux + dy*suy;
    dv = dx*svx + dy*svy;
    d3 = (fabs(du) > fabs(dv)) ? ((du>0)?u:vneg(u)) : ((dv>0)?v:vneg(v));
    ax = vcross(n, d3);
    a = vaxis(ax);
    m.axis = a;
    m.k = vc(ax,a);
    m.layer = vc(slots[idx].p, a);
    return m;
}

/* -------------------------------- 命中测试 */
static int in_quad(POINT p, POINT q[4])
{
    int i, j, c = 0;
    for (i=0,j=3;i<4;j=i++) {
        if (((q[i].y > p.y) != (q[j].y > p.y)) &&
            (p.x < (double)(q[j].x-q[i].x)*(p.y-q[i].y)/(double)(q[j].y-q[i].y) + q[i].x))
            c = !c;
    }
    return c;
}
static int hit_sticker(int x, int y)
{
    int i; POINT p; p.x=x; p.y=y;
    for (i=nitems-1;i>=0;i--)                /* 从近到远 */
        if (in_quad(p, items[i].q)) return items[i].idx;
    return -1;
}
/* 左键落在舞台空白处时, 取最近的可见贴纸作为转层基准 —— 左键永远只转魔方 */
static int nearest_sticker(int x, int y)
{
    int i, j, best=-1;
    double bd=1e30, cap=2.6*g_scale;
    cap *= cap;
    for (i=0;i<nitems;i++) {
        double cx=0, cy=0, dx, dy, d;
        for (j=0;j<4;j++) { cx+=items[i].q[j].x; cy+=items[i].q[j].y; }
        cx/=4.0; cy/=4.0;
        dx=cx-x; dy=cy-y; d=dx*dx+dy*dy;
        if (d<bd) { bd=d; best=items[i].idx; }
    }
    return (bd<=cap) ? best : -1;
}
static int hit_button(int x, int y)
{
    int i; POINT p; p.x=x; p.y=y;
    for (i=0;i<NBTN;i++) if (PtInRect(&btns[i].rc,p)) return btns[i].id;
    return 0;
}

/* -------------------------------- 布局 / 按钮表 */
static void init_buttons(void)
{
    btns[0].label=L"主视角"; btns[0].id=ID_V0;
    btns[1].label=L"打乱";   btns[1].id=ID_SHUF;
    btns[2].label=L"复位";   btns[2].id=ID_RESET;
    btns[3].label=L"说明书"; btns[3].id=ID_HELP;
    btns[4].label=L"输入";   btns[4].id=ID_RUN;
}
static void layout(int W, int H)
{
    static const int bw[NBTN] = { 58, 46, 46, 58, 46 };
    int row_h=28, top_h=54;
    int row_y = H - row_h - 10;
    int stage_bottom = row_y - 8;           /* 底部没有状态行了 */
    int x, sw, sh, edit_right, i;

    if (W < 200 || H < 200) return;
    g_w=W; g_h=H;
    SetRect(&g_stage_rect, 12, top_h+4, W-12, stage_bottom);
    if (g_stage_rect.bottom < g_stage_rect.top+120)
        g_stage_rect.bottom = g_stage_rect.top+120;
    sw = g_stage_rect.right-g_stage_rect.left;
    sh = g_stage_rect.bottom-g_stage_rect.top;
    g_cx = (g_stage_rect.left+g_stage_rect.right)/2.0;
    g_cy = (g_stage_rect.top+g_stage_rect.bottom)/2.0 + 6;
    g_scale = (double)(sw < sh ? sw : sh) / 6.2;
    if (g_scale < 24) g_scale = 24;

    /* 底部唯一一行: [主视角] [打乱] [复位] [说明书]  指令: [输入框......] [输入] */
    x = 12;
    for (i=0;i<4;i++) {
        SetRect(&btns[i].rc, x, row_y+1, x+bw[i], row_y+row_h-1);
        x += bw[i] + 6;
    }
    x += 4;
    SetRect(&g_label_rc, x, row_y, x+42, row_y+row_h);
    x += 42;
    edit_right = W-12-bw[4]-6;
    if (edit_right < x+40) edit_right = x+40;
    SetRect(&g_edit_rc, x, row_y+2, edit_right, row_y+row_h-2);
    SetRect(&btns[4].rc, W-12-bw[4], row_y+1, W-12, row_y+row_h-1);
    if (g_hedit)
        MoveWindow(g_hedit, g_edit_rc.left, g_edit_rc.top,
                   g_edit_rc.right-g_edit_rc.left,
                   g_edit_rc.bottom-g_edit_rc.top, TRUE);
}

/* -------------------------------- 主绘制 */
static void draw(HDC dc)
{
    RECT rc;
    int i, steps = hpos - hist_base;
    if (steps < 0) steps = 0;

    fr(dc,0,0,g_w,g_h,g_brBg);

    /* 顶部: 扫雷式计数器 */
    {
        int lx = (int)g_cx - 190;
        int tx = (int)g_cx + 92;
        SetRect(&rc,lx-52,20,lx-4,46);
        txt(dc,L"步数",rc,g_fUI,C_BLACK,DT_RIGHT|DT_VCENTER|DT_SINGLELINE);
        led3(dc,lx,16,steps);
        SetRect(&rc,tx-52,20,tx-4,46);
        txt(dc,L"时间",rc,g_fUI,C_BLACK,DT_RIGHT|DT_VCENTER|DT_SINGLELINE);
        led3(dc,tx,16,(int)(now_elapsed()/1000));
    }

    /* 3D 舞台: 凹陷面板 */
    {
        int x=g_stage_rect.left, y=g_stage_rect.top;
        int w=g_stage_rect.right-x, h=g_stage_rect.bottom-y;
        fr(dc,x,y,w,h,g_brStage);
        bevel(dc,x,y,w,h,0);
        {
            int saved = SaveDC(dc);      /* 动画中转动的层可能越界, 限制在舞台内 */
            IntersectClipRect(dc,x+2,y+2,x+w-2,y+h-2);
            draw_cube(dc);
            RestoreDC(dc,saved);
        }
    }

    /* 底部唯一一行: 四个按钮 + 指令标签 + 输入框 + 输入按钮 */
    txt(dc,L"指令:",g_label_rc,g_fUI,C_BLACK,DT_RIGHT|DT_VCENTER|DT_SINGLELINE);
    for (i=0;i<NBTN;i++) {
        Btn *b=&btns[i];
        int x=b->rc.left,y=b->rc.top,w=b->rc.right-x,h=b->rc.bottom-y;
        int pressed = (press_id==b->id);
        RECT tr=b->rc;
        fr(dc,x,y,w,h,g_brBg);
        bevel(dc,x,y,w,h,!pressed);
        if (pressed) { tr.left++; tr.top++; tr.right++; tr.bottom++; }
        txt(dc,b->label,tr,g_fUI,C_BLACK,DT_CENTER|DT_VCENTER|DT_SINGLELINE);
    }
}
static void repaint(void)
{
    PAINTSTRUCT ps;
    HDC dc = BeginPaint(g_hwnd,&ps);
    int w = g_w>1?g_w:1, h = g_h>1?g_h:1;
    if (!g_memdc) {
        HDC sdc = GetDC(g_hwnd);
        g_memdc = CreateCompatibleDC(sdc);
        g_membmp = CreateCompatibleBitmap(sdc,w,h);
        g_oldbmp = (HBITMAP)SelectObject(g_memdc,g_membmp);
        ReleaseDC(g_hwnd,sdc);
    }
    draw(g_memdc);
    BitBlt(dc,0,0,w,h,g_memdc,0,0,SRCCOPY);
    EndPaint(g_hwnd,&ps);
}
static void resize_buffer(void)
{
    if (g_memdc) {
        SelectObject(g_memdc,g_oldbmp);
        DeleteObject(g_membmp); DeleteDC(g_memdc); g_memdc=NULL;
    }
}

/* -------------------------------- 动作 */
static void show_help(void)
{
    MessageBoxW(g_hwnd,
        L"【鼠标】左键拖动 = 只转动魔方：按在贴纸上拖 = 转该层；舞台空白处拖 = 转最近的层。\n"
        L"        右键拖动 = 只转视角；向下拖抬头、向上拖低头，左右方向不变。\n"
        L"        左键绝不会改变视角，右键绝不会转动魔方，视角不进指令通道。\n"
        L"【按钮】主视角 = 恢复默认斜视角；打乱 / 复位；说明书 = 本窗口；输入 = 执行指令。\n"
        L"【指令】在输入框输入后按 Enter 或点「输入」，指令只负责转动魔方。\n"
        L"        U D L R F B = 六个面，M E S = 三个中间切片；\n"
        L"        小写 = 反向，' = 反向，2 = 转 180°；空格分隔或连写。\n"
        L"        例：R U R' U' F2 M        reset = 直接复位并清零步数/计时。\n"
        L"【打乱】「打乱」/ F2：生成 22 条随机指令填入输入框，经同一指令通道播放，可复制复现。\n"
        L"【计时】打乱后第一步自动开始，六面同色停表；进度自动存盘，重开继续。\n"
        L"【其它】F1 说明  ·  F2 打乱  ·  Esc 中止播放  ·  Enter 聚焦输入框；窗口大小固定。",
        L"操作说明 - CUBE95", MB_OK|MB_ICONINFORMATION);
}
static void do_cmd_id(int id)
{
    if (id==ID_V0)        { g_tYaw=-0.62; g_tPitch=0.48; }
    else if (id==ID_SHUF) shuffle();
    else if (id==ID_RESET) do_reset();
    else if (id==ID_HELP)  show_help();
    else if (id==ID_RUN)   execute_from_edit();
    InvalidateRect(g_hwnd,NULL,FALSE);
}
/* 输入框子类: 单行 EDIT 默认不处理 Enter, 这里拦下来执行指令 */
static LRESULT CALLBACK edit_proc(HWND h, UINT m, WPARAM w, LPARAM l)
{
    if (m==WM_KEYDOWN && w==VK_RETURN) { execute_from_edit(); return 0; }
    if (m==WM_KEYDOWN && w==VK_ESCAPE) { SetFocus(g_hwnd); return 0; }
    return CallWindowProcW(g_edit_old,h,m,w,l);
}

/* -------------------------------- 窗口过程 */
static LRESULT CALLBACK wnd_proc(HWND hwnd,UINT msg,WPARAM wp,LPARAM lp)
{
    switch(msg) {
    case WM_CREATE: {
        LPCREATESTRUCTW cs = (LPCREATESTRUCTW)lp;
        g_hedit = CreateWindowExW(WS_EX_CLIENTEDGE, L"EDIT", L"",
            WS_CHILD|WS_VISIBLE|WS_TABSTOP|ES_AUTOHSCROLL|ES_LEFT,
            0,0,10,10,hwnd,(HMENU)(UINT_PTR)ID_EDIT,cs->hInstance,NULL);
        if (g_hedit) {
            SendMessageW(g_hedit, WM_SETFONT, (WPARAM)g_fUI, TRUE);
            SendMessageW(g_hedit, EM_LIMITTEXT, 511, 0);
            SendMessageW(g_hedit, EM_SETCUEBANNER, TRUE,
                (LPARAM)L"例如  R U R' U' F2   ·   reset 复位");
            g_edit_old = (WNDPROC)SetWindowLongPtrW(g_hedit,GWLP_WNDPROC,
                                                    (LONG_PTR)edit_proc);
        }
        SetTimer(hwnd,1,16,NULL);
        return 0;
    }

    case WM_SIZE:
        layout(LOWORD(lp),HIWORD(lp));
        resize_buffer();
        InvalidateRect(hwnd,NULL,FALSE);
        return 0;

    case WM_GETMINMAXINFO: {            /* 固定尺寸: 拖边框 / 双击标题栏都不变 */
        MINMAXINFO *mi = (MINMAXINFO*)lp;
        mi->ptMinTrackSize.x = mi->ptMaxTrackSize.x = WIN_W;
        mi->ptMinTrackSize.y = mi->ptMaxTrackSize.y = WIN_H;
        return 0;
    }

    case WM_COMMAND:
        if (LOWORD(wp)==ID_RUN) { execute_from_edit(); return 0; }
        return 0;

    case WM_CTLCOLOREDIT:
    case WM_CTLCOLORSTATIC:
        SetTextColor((HDC)wp,C_BLACK);
        SetBkColor((HDC)wp,C_WHITE);
        return (LRESULT)g_brWhite;

    case WM_ERASEBKGND: return 1;
    case WM_PAINT:      repaint(); return 0;

    case WM_LBUTTONDOWN:
    case WM_RBUTTONDOWN: {
        int x=GET_X_LPARAM(lp), y=GET_Y_LPARAM(lp);
        int id = (msg==WM_LBUTTONDOWN) ? hit_button(x,y) : 0;
        POINT p; p.x=x; p.y=y;
        if (id) { press_id=id; SetCapture(hwnd); return 0; }
        drag_done=0; drag_pt.x=x; drag_pt.y=y; drag_last.x=x; drag_last.y=y;
        if (msg==WM_RBUTTONDOWN) {          /* 右键: 一律转视角 */
            drag_mode=2; drag_idx=-1; SetCapture(hwnd);
        } else if (PtInRect(&g_stage_rect,p)) {
            /* 左键: 一律转魔方, 绝不影响视角 */
            drag_idx = hit_sticker(x,y);
            if (drag_idx<0) drag_idx = nearest_sticker(x,y);
            drag_mode = (drag_idx>=0) ? 1 : 0;
            if (drag_mode) SetCapture(hwnd);
        } else drag_mode=0;
        SetFocus(hwnd);
        return 0;
    }

    case WM_MOUSEMOVE: {
        int x=GET_X_LPARAM(lp), y=GET_Y_LPARAM(lp);
        if (drag_mode==1 && !drag_done) {
            if (abs(x-drag_pt.x)+abs(y-drag_pt.y) >= 12) {
                Cmd c; c.m=drag_to_move(drag_idx,x-drag_pt.x,y-drag_pt.y);
                c.kind=K_MOVE; enqueue(c);
                drag_done=1;
                InvalidateRect(hwnd,NULL,FALSE);
            }
        } else if (drag_mode==2) {
            view_drag(x-drag_last.x, y-drag_last.y, &g_tYaw, &g_tPitch);
            drag_last.x=x; drag_last.y=y;
            InvalidateRect(hwnd,NULL,FALSE);
        }
        return 0;
    }

    case WM_LBUTTONUP:
    case WM_RBUTTONUP:
        ReleaseCapture();
        if (msg==WM_LBUTTONUP && press_id) {
            POINT p; p.x=GET_X_LPARAM(lp); p.y=GET_Y_LPARAM(lp);
            int id = press_id;
            if (hit_button(p.x,p.y)==id) do_cmd_id(id);
        }
        press_id=0; drag_mode=0; drag_done=0;
        InvalidateRect(hwnd,NULL,FALSE);
        return 0;

    case WM_KEYDOWN:
        switch(wp) {
        case VK_F1:      show_help(); return 0;
        case VK_F2:      shuffle(); return 0;
        case VK_RETURN:  SetFocus(g_hedit);
                         SendMessageW(g_hedit,EM_SETSEL,0,-1); return 0;
        case VK_ESCAPE:  clear_queue(); set_status(L"已中止播放"); return 0;
        }
        break;

    case WM_TIMER: {
        DWORD now=GetTickCount();
        /* 标题栏短提示到期自动消失 */
        if (g_msg[0] && now > g_msg_until) {
            g_msg[0]=0; SetWindowTextW(hwnd,APP_TITLE);
        }
        /* 视角平滑跟随 */
        g_yaw   += (g_tYaw-g_yaw)*0.22;
        g_pitch += (g_tPitch-g_pitch)*0.22;
        /* 动画结束 -> 取下一条命令 */
        if (anim.on && now-anim.t0 >= (DWORD)anim.dur) anim.on=0;
        if (!anim.on && qh<qt) apply_cmd(queue[qh++ % QCAP]);
        /* 打乱播放完成: 这 22 步记为基准, 不计入用户步数 */
        if (scram_pending && qh==qt && !anim.on) {
            hist_base=hlen; scram_pending=0; save_state();
        }
        InvalidateRect(hwnd,NULL,FALSE);
        return 0;
    }

    case WM_DESTROY:
        save_state();
        KillTimer(hwnd,1);
        resize_buffer();
        {
            int i;
            for (i=0;i<NF;i++) DeleteObject(g_brFace[i]);
            DeleteObject(g_brBg); DeleteObject(g_brWhite); DeleteObject(g_brHi);
            DeleteObject(g_brShade); DeleteObject(g_brDark); DeleteObject(g_brBlack);
            DeleteObject(g_brStage); DeleteObject(g_brCube);
            DeleteObject(g_brLedOn); DeleteObject(g_brLedOff);
            DeleteObject(g_pnBlack); DeleteObject(g_pnThin);
            DeleteObject(g_fUI); DeleteObject(g_fSmall);
        }
        PostQuitMessage(0);
        return 0;
    }
    return DefWindowProcW(hwnd,msg,wp,lp);
}

static HFONT mkfont(int px, int bold)
{
    return CreateFontW(-px,0,0,0,bold?FW_BOLD:FW_NORMAL,FALSE,FALSE,FALSE,
                       DEFAULT_CHARSET,OUT_DEFAULT_PRECIS,CLIP_DEFAULT_PRECIS,
                       DEFAULT_QUALITY,DEFAULT_PITCH|FF_DONTCARE,L"MS Sans Serif");
}

int WINAPI WinMain(HINSTANCE hInst,HINSTANCE hPrev,LPSTR cmd,int show)
{
    WNDCLASSW wc; MSG msg; int i;
    (void)hPrev; (void)cmd; (void)show;

    init_geometry();
    clear_game();
    load_state();
    if (!counts_ok()) clear_game();
    init_buttons();
    srand((unsigned)(GetTickCount() ^ (unsigned)time(NULL) ^
                    (unsigned)(ULONG_PTR)GetCurrentProcessId()));

    for (i=0;i<NF;i++) g_brFace[i]=CreateSolidBrush(FACE_COLOR[i]);
    g_brBg    = CreateSolidBrush(C_BG);
    g_brWhite = CreateSolidBrush(C_WHITE);
    g_brHi    = CreateSolidBrush(C_HI);
    g_brShade = CreateSolidBrush(C_SHADE);
    g_brDark  = CreateSolidBrush(C_DARK);
    g_brBlack = CreateSolidBrush(C_BLACK);
    g_brStage = CreateSolidBrush(C_STAGE);
    g_brCube  = CreateSolidBrush(C_CUBE);
    g_brLedOn = CreateSolidBrush(C_LED_ON);
    g_brLedOff= CreateSolidBrush(C_LED_OFF);
    g_pnBlack = CreatePen(PS_SOLID,2,C_BLACK);
    g_pnThin  = CreatePen(PS_SOLID,1,C_BLACK);
    g_fUI     = mkfont(13,0);
    g_fSmall  = mkfont(12,0);

    ZeroMemory(&wc,sizeof(wc));
    wc.lpfnWndProc=wnd_proc;
    wc.hInstance=hInst;
    wc.lpszClassName=L"CUBE95_Rubiks";
    wc.hCursor=LoadCursor(NULL,IDC_ARROW);
    wc.hIcon=LoadIcon(NULL,IDI_APPLICATION);
    wc.lpszMenuName=NULL;                 /* 无菜单栏 */
    wc.style=CS_HREDRAW|CS_VREDRAW;
    if (!RegisterClassW(&wc)) return 1;

    {
        /* 无 WS_THICKFRAME / WS_MAXIMIZEBOX: 窗口不可缩放、不可最大化 */
        DWORD style = WS_OVERLAPPED|WS_CAPTION|WS_SYSMENU|WS_MINIMIZEBOX|WS_CLIPCHILDREN;
        int sx = (GetSystemMetrics(SM_CXSCREEN)-WIN_W)/2;
        int sy = (GetSystemMetrics(SM_CYSCREEN)-WIN_H)/2;
        if (sx<0) sx=0;
        if (sy<0) sy=0;
        g_hwnd=CreateWindowExW(0,L"CUBE95_Rubiks",APP_TITLE,style,
            sx,sy,WIN_W,WIN_H,NULL,NULL,hInst,NULL);
    }
    if (!g_hwnd) return 1;
    ShowWindow(g_hwnd,SW_SHOWNORMAL);
    UpdateWindow(g_hwnd);

    while (GetMessageW(&msg,NULL,0,0)>0) {
        TranslateMessage(&msg);
        DispatchMessageW(&msg);
    }
    return (int)msg.wParam;
}

#else  /* =================================================== 逻辑自检 */
static int run_selftest(void)
{
    int fails=0,f,d,i,t;
    unsigned char snapshot[NS];
    Move seq[600], mv[SCRAM_N], mv2[SCRAM_N], parsed[16];
    WCHAR txt[256];
#define CHECK(cond,name) do { int ok=(cond); \
    printf("  [%s] %s\n", ok?"PASS":"FAIL", name); if(!ok) fails++; } while(0)

    printf("CUBE95 3D 核心逻辑自检\n");
    logic_error=0;
    /* 这些函数只在 GUI 版使用, 此处引用一下保持零警告编译 */
    (void)cam_rot; (void)p3add; (void)p3scale;
    (void)start_clock; (void)now_elapsed; (void)logic_redo;
    init_geometry(); init_solved();
    CHECK(counts_ok(),"初始状态每色 9 张");
    CHECK(is_solved(),"初始状态六面同色");
    {
        int good=1;
        for (i=0;i<NS;i++) if (find_slot(slots[i].p,slots[i].n)!=i) good=0;
        CHECK(good,"54 个槽位几何唯一");
    }
    {
        int good=1;
        for (f=0;f<NF;f++) if (!veq(vcross(VVEC[f],UVEC[f]),NORM[f])) good=0;
        CHECK(good,"六面切向量满足 n = v x u");
    }
    {
        /* 浮点任意角旋转在 ±90 度时必须与整数 rot90 完全一致 */
        int good=1, a, j;
        const V3 vs[4]={{1,0,0},{0,1,-1},{1,1,0},{-1,0,1}};
        for (a=0;a<3;a++) for (j=0;j<4;j++) for (t=-1;t<=1;t+=2) {
            V3 iv=rot90(vs[j],a);
            P3 fv=rot_axis(p3v(vs[j]),a,t*PI/2.0);
            if (fabs(fv.x-iv.x)>1e-9 || fabs(fv.y-iv.y)>1e-9 || fabs(fv.z-iv.z)>1e-9) {
                /* t=-1 时应等于反方向 90 度, 单独校验 */
                if (t==1) good=0;
                else { V3 inv=rot_vec(vs[j],a,3);
                       if (fabs(fv.x-inv.x)>1e-9||fabs(fv.y-inv.y)>1e-9||fabs(fv.z-inv.z)>1e-9) good=0; }
            }
        }
        CHECK(good,"浮点动画旋转与整数转层在 90 度处严格一致");
    }
    /* ---------- 视角拖拽映射 ---------- */
    {
        double y=0.0, p=0.0;
        view_drag(100,0,&y,&p);
        CHECK(y>0 && p==0.0,"视角左右: 右拖 yaw 增大 (保持原方向)");
        view_drag(-200,0,&y,&p);
        CHECK(y<0,"视角左右: 左拖 yaw 减小");
        y=0.0; p=0.0;
        view_drag(0,100,&y,&p);
        CHECK(y==0.0 && p>0,"视角上下: 下拖 pitch 增大 (上下已反转)");
        view_drag(0,-300,&y,&p);
        CHECK(p<0,"视角上下: 上拖 pitch 减小");
        view_drag(0,1000,&y,&p);
        CHECK(p>1.2499,"视角俯仰上限钳制 +1.25");
        view_drag(0,-1000,&y,&p);
        CHECK(p<-1.2499,"视角俯仰下限钳制 -1.25");
    }
    /* ---------- 指令通道 ---------- */
    {
        int n = parse_commands(L"R U R' U' F2 M E S'", parsed, 16);
        int good = (n==8);
        if (good) {
            Move e[8];
            e[0]=face_move(F_R,0); e[1]=face_move(F_U,0);
            e[2]=face_move(F_R,1); e[3]=face_move(F_U,1);
            e[4]=face_move(F_F,0); e[4].k=2;
            e[5].axis=0; e[5].layer=0; e[5].k=-1;
            e[6].axis=1; e[6].layer=0; e[6].k=-1;
            e[7].axis=2; e[7].layer=0; e[7].k= 1;
            for (i=0;i<8;i++)
                if (parsed[i].axis!=e[i].axis||parsed[i].layer!=e[i].layer||parsed[i].k!=e[i].k) good=0;
        }
        CHECK(good,"指令串 R U R' U' F2 M E S' 解析为 8 个正确转动");
        CHECK(parse_commands(L"u d l r f b",parsed,16)==6 &&
              parsed[0].k==-face_move(F_U,0).k,"小写字母 = 反向转动");
        CHECK(parse_commands(L"R U ? F",parsed,16)==-1,"非法字符被拒绝且整串不执行");
        CHECK(parse_commands(L"R U R U",parsed,3)==-2,"超出上限返回 -2");
        CHECK(parse_commands(L"RUR'U'",parsed,16)==4,"连写 RUR'U' 同样可解析");
        CHECK(is_reset_cmd(L"  ReSeT \t") && !is_reset_cmd(L"reset2") &&
              !is_reset_cmd(L"R E S E T"),"reset 指令大小写不敏感且严格匹配");
    }
    {
        /* 打乱生成的随机指令文本, 经命令行解析后必须逐条无损还原 */
        int n, good=1;
        srand(777);
        make_scramble(mv);
        build_scramble_text(mv,txt);
        n = parse_commands(txt,mv2,SCRAM_N);
        if (n!=SCRAM_N) good=0;
        else for (i=0;i<SCRAM_N;i++)
            if (mv[i].axis!=mv2[i].axis||mv[i].layer!=mv2[i].layer||mv[i].k!=mv2[i].k) good=0;
        CHECK(good,"打乱随机指令文本经指令通道解析后逐步一致 (含 2)");
    }
    for (f=0;f<NF;f++) {
        Move m=face_move(f,0); WCHAR nm[8]; char ansi[64];
        init_solved();
        for (t=0;t<4;t++) rotate_layer(m.axis,m.layer,m.k);
        move_notation(m,nm);
        sprintf(ansi,"%c%s 转 4 次复原",(char)nm[0],nm[1]?(nm[1]==L'\''?"'":"2"):"");
        CHECK(is_solved()&&counts_ok(),ansi);
    }
    for (f=0;f<NF;f++) {
        Move m=face_move(f,1); WCHAR nm[8]; char ansi[64];
        init_solved();
        for (t=0;t<4;t++) rotate_layer(m.axis,m.layer,m.k);
        move_notation(m,nm);
        sprintf(ansi,"%c%s 转 4 次复原",(char)nm[0],nm[1]?(nm[1]==L'\''?"'":"2"):"");
        CHECK(is_solved()&&counts_ok(),ansi);
    }
    {
        int good=1,good2=1;
        for (f=0;f<NF;f++) {
            V3 dirs[4]; dirs[0]=UVEC[f]; dirs[1]=vneg(UVEC[f]);
            dirs[2]=VVEC[f]; dirs[3]=vneg(VVEC[f]);
            for (d=0;d<4;d++) {
                V3 axv=vcross(NORM[f],dirs[d]);
                int a=vaxis(axv), k=vc(axv,a);
                if (!veq(rot_vec(NORM[f],a,k),dirs[d])) good=0;
                init_solved();
                for (t=0;t<4;t++) rotate_layer(a,0,k);
                if (!is_solved()||!counts_ok()) good2=0;
            }
        }
        CHECK(good,"24 种拖拽方向 -> 贴纸沿拖动方向翻面");
        CHECK(good2,"24 种拖拽方向 -> 转 4 次回到原状");
    }
    init_solved();
    for (i=0;i<NS;i++) snapshot[i]=slots[i].color;
    srand(20261105);
    for (i=0;i<600;i++) {
        seq[i].axis=rand()%3; seq[i].layer=rand()%3-1; seq[i].k=(rand()&1)?1:-1;
        rotate_layer(seq[i].axis,seq[i].layer,seq[i].k);
        if (!counts_ok()) break;
    }
    CHECK(counts_ok()&&!logic_error,"随机 600 步颜色守恒 / 无非法槽位");
    for (i=599;i>=0;i--) rotate_layer(seq[i].axis,seq[i].layer,-seq[i].k);
    {
        int good=1;
        for (i=0;i<NS;i++) if (slots[i].color!=snapshot[i]) good=0;
        CHECK(good&&is_solved(),"600 步逐步逆序 = 逐格精确复原");
    }
    /* 打乱 -> 用户操作 -> 普通撤销 -> 全历史回放 */
    clear_game();
    make_scramble(mv);
    build_scramble_text(mv,txt);
    for (i=0;i<SCRAM_N;i++) { rotate_layer(mv[i].axis,mv[i].layer,mv[i].k); logic_push_base(mv[i]); }
    hist_base=hlen;
    CHECK(counts_ok()&&!is_solved(),"打乱 22 步后颜色守恒且已乱序");
    for (i=0;i<NS;i++) snapshot[i]=slots[i].color;
    for (i=0;i<100;i++) {
        Move m=face_move(rand()%6,rand()&1);
        rotate_layer(m.axis,m.layer,m.k); logic_push(m);
    }
    CHECK(hpos-hist_base==100,"打乱步不计入用户步数");
    while (1) { Move m=logic_undo(hist_base); if(!m.k)break; rotate_layer(m.axis,m.layer,m.k); }
    {
        int good=1;
        for (i=0;i<NS;i++) if (slots[i].color!=snapshot[i]) good=0;
        CHECK(good,"100 步撤销 = 精确回到打乱完成状态");
        while (1) { Move m=logic_undo(0); if(!m.k)break; rotate_layer(m.axis,m.layer,m.k); }
        CHECK(is_solved()&&counts_ok(),"全历史逆序回放 = 六面复原");
    }
    printf("\n%s (失败项: %d)\n", fails?"== 自检失败 ==":"== 全部自检通过 ==",fails);
    return fails;
}
int main(void) { return run_selftest()?1:0; }
#endif
