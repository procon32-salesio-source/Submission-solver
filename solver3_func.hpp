#ifndef INCLUDE_GUARD_SOLVER2_FUNC
#define INCLUDE_GUARD_SOLVER2_FUNC

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <math.h>
#include <random>
#include <typeinfo>
using namespace std;
#define INF 100000000
random_device rnd;
mt19937 mt(rnd());

char hx(int x){ //10進を16進に
    char r;
    if(x <= 9) r = x+'0';
    else r = x+'A'-10;
    return r;
}

int xh(char c){ //16進を10進に
    int x;
    if(c <= '9') x = c-'0';
    else x = (c<'G') ? (c-'A'+10) : (c-'a'+10);
    return x;
}

typedef struct ido{ //盤面を管理する構造体
    int gox;
    int goy;
    int rot;
} Ido;

struct Node{ //beamサーチのノードを管理する構造体
    vector<vector<Ido>> map;
    int choose_x;
    int choose_y;
    int choose_limit;
    int cost_chos;
    int cost_swap;
    char status; //N -> none / O -> open / C -> closed
    int real_cost;
    double heur_cost; //heuristic(推定) cost
    Node* parent;
    vector<char> how_move; //parentをどう操作するとこのNodeになるか (U,D,L,R,o)
    int chooses; //選択回数
    vector<Ido> how_choose; //n回目の選択でどこに移動したか
};

struct Line{ //回答フォーマット作成のための構造体
    char pozx;
    char pozy;
    int cnt;
    string sosa;
};

void swap(Ido &a, Ido &b){ //swap関数
    Ido tmp;
    tmp = a;
    a = b;
    b = tmp;
}

void swap(vector<vector<Ido>> &m, Node &a, Node &b){ //swap関数
    Node tmp;
    tmp = a;
    a = b;
    b = tmp;
}

void init(Node &n, vector<vector<Ido>> &m, int x, int y, int c, int cc, int cs){ //初期化関数
    n.map = m;
    n.choose_x = x;
    n.choose_y = y;
    n.choose_limit = c;
    n.cost_chos = cc;
    n.cost_swap = cs;
    n.status = 'N';
    n.real_cost = 0;
    n.heur_cost = INF;
    n.parent = NULL;
    n.how_move.push_back('o');
    n.chooses = 1;
    n.how_choose.push_back({x,y,0});
}

bool isGoal(vector<vector<Ido>> &m){ //ゴールしているか しているなら1
    for(int j = 0; j < m.size(); j++){
        for(int i = 0; i < m[0].size(); i++){
            if(m[j][i].gox != i || m[j][i].goy != j) return 0;
        }
    }
    return 1;
}

void prin(Node &n){ //Nodeを簡単に出力
    cout << "real cost : " << n.real_cost << endl;
    cout << "heur cost : " << n.heur_cost << endl;
    cout << "choosings : " << n.chooses << endl << endl;
}

void prin2(Node &n){ //Nodeをわかりやすく出力
    int y = int(n.map.size()), x = int(n.map[0].size());
    cout << "map size -> x : " << x << ", y : " << y << endl;
    cout << "choosing -> x : " << n.choose_x << ", y : " << n.choose_y << endl;
    for(int i = 0; i < y; i++){
        for(int j = 0; j < x; j++)
            cout << "[" << n.map[i][j].gox << ", " << n.map[i][j].goy << ", " << n.map[i][j].rot << "] ";
        cout << endl;
    }
    cout << endl;
    cout << "        status : ";
    if(n.status == 'N') cout << "None" << endl;
    else if(n.status == 'O') cout << "Open" << endl;
    else cout << "Closed" << endl;
    cout << "     real cost : " << n.real_cost << endl;
    cout << "heuristic cost : " << n.heur_cost << endl;
    cout << "        parent : ";
    if(n.parent == NULL) cout << "NULL" << endl;
    else cout << n.parent << endl;
    for(int i = 0; i < n.how_move.size(); i++) cout << n.how_move[i];
    cout << endl << n.chooses << " : ";
    for(int i = 0; i < n.how_choose.size(); i++) cout << n.how_choose[i].gox << n.how_choose[i].goy << " ";
    cout << endl;
    if(isGoal(n.map)) cout << "Happy" << endl;
    else cout << "No" << endl;
}

int manhattan(vector<vector<Ido>> &m, int x, int y){ //マンハッタン距離を求める トーラスを考慮
    int sizex = int(m[0].size()), sizey = int(m.size());
    int nx = m[y][x].gox, ny = m[y][x].goy;
    int n;
    n = min(abs(nx - x), sizex - abs(nx - x)) + min(abs(ny - y), sizey - abs(ny - y));
    return n;
}

Ido firstposition(vector<vector<Ido>> &m){ //初期位置を求める 一番遠いブロック
    int maxdis = 0;
    Ido d = {0, 0, 0};
    for(int j = 0; j < m.size(); j++){
        for(int i = 0; i < m[0].size(); i++){
            if(manhattan(m, i, j) > maxdis){
                maxdis = manhattan(m, i, j);
                d.gox = i;
                d.goy = j;
            }
        }
    }
    return d;
}

bool parity(vector<vector<Ido>> &m, int x, int y){ //1選択で完成するか 可能なら1
    int sizex = int(m[0].size()), sizey = int(m.size()), a = 0, even = 0;
    a = m[y][x].gox + m[y][x].goy * sizex;
    if(sizex % 2 == 1 || sizey % 2 == 1) return 1;
    vector<int> root(sizex*sizey), path(sizex*sizey);
    for(int j = 0; j < sizey; j++){
        for(int i = 0; i < sizex; i++){
            if((sizey+j+1)&1){
                root[i+j*sizex] = sizex-i-1+j*sizex;
                path[i+j*sizex] = m[j][sizex-i-1].gox + m[j][sizex-i-1].goy * sizex;
            }
            else{
                root[i+j*sizex] = i+j*sizex;
                path[i+j*sizex] = m[j][i].gox + m[j][i].goy * sizex;
            }
        }
    }
    
    for(int i = 0; i < path.size(); i++) if(path[i] == a) path.erase(path.begin() + i);
    for(int i = 0; i < root.size(); i++) if(root[i] == a) root.erase(root.begin() + i);
    for(int i = 0; i < root.size(); i++){
        for(int j = 0; j < path.size(); j++){
            if(path[j] == root[i]) {
                path.erase(path.begin() + j);
                even += j;
                break;
            }
        }
    }
    
    return (even % 2) == 0;
}

Ido all_parity(vector<vector<Ido>> &m){ //初期位置の決定 1選択でできるようなブロックのうち、一番遠いやつ
    int sizex = int(m[0].size()), sizey = int(m.size()), maxm = -1;
    Ido d = {-1,-1,0}; //が一個もない時は-1を返す
    for(int j = 0; j < sizey; j++){
        for(int i = 0; i < sizex; i++){
            if(parity(m, i, j) && manhattan(m, i, j) > maxm){
                d.gox = i;
                d.goy = j;
                maxm = manhattan(m, i, j);
            }
        }
    }
//    if(all == sizex*sizey) return 10;
//    else if(all) return 1;
//    else return 0;
    return d;
}

Ido where(vector<vector<Ido>> &m, int x, int y){
    Ido d = {0,0,0};
    for(int j = 0; j < m.size(); j++){
        for(int i = 0; i < m[0].size(); i++){
            if(m[j][i].gox == x && m[j][i].goy == y){
                d.gox = i;
                d.goy = j;
                return d;
            }
        }
    }
    return d;
}

void heuristic(Node &n){ //heuristic(推定)関数 精度が良いほど短い手で答えが出る
    int m = 0;
    double  md = 0, smd = 0, r = 0;
    n.heur_cost = 0;
//    n.heur_cost = n.how_move.size();
//    if(csiz > n.choose_limit) n.heur_cost += INF;
//    if(parity(n.map, n.choose_x, n.choose_y)) n.heur_cost += 20;
    for(int j = 0; j < n.map.size(); j++){
        for(int i = 0; i < n.map[0].size(); i++){
            m = manhattan(n.map, i, j);
            md += m / 1.2;
            smd += m*m;
        }
    }
    r = (int(mt() % 7) - 3) / 2.0;
    smd = sqrt(smd) / 4.0;
    n.heur_cost += md + smd + r;
}

void U(vector<vector<Ido>> &m, int &x, int &y){ //Up関数
    int py = y;
    if(y != 0) y--;
    else y = int(m.size()-1);
    swap(m[py][x], m[y][x]);
}

void D(vector<vector<Ido>> &m, int &x, int &y){ //Down関数
    int py = y;
    if(y != m.size()-1) y++;
    else y = 0;
    swap(m[py][x], m[y][x]);
}

void L(vector<vector<Ido>> &m, int &x, int &y){ //Left関数
    int px = x;
    if(x != 0) x--;
    else x = int(m[0].size()-1);
    swap(m[y][px], m[y][x]);
}

void R(vector<vector<Ido>> &m, int &x, int &y){ //Right関数
    int px = x;
    if(x != m[0].size()-1) x++;
    else x = 0;
    swap(m[y][px], m[y][x]);
}

void move(Node &n, char c, Node &n2){ //親ノードを1手交換した子ノードを作成
    n2 = n;
    n2.real_cost++;
    n2.how_move.push_back(c);
    n2.parent = &n;
    n2.status = 'O';
    if(c == 'U') U(n2.map, n2.choose_x, n2.choose_y);
    if(c == 'D') D(n2.map, n2.choose_x, n2.choose_y);
    if(c == 'R') R(n2.map, n2.choose_x, n2.choose_y);
    if(c == 'L') L(n2.map, n2.choose_x, n2.choose_y);
    heuristic(n2);
}

void move_c(Node &n, Node &n2){ //親ノードを1手選択した子ノードを作成
    n2 = n;
    Ido d = {0,0,0};
    n2.chooses++;
    if(n2.chooses >= n2.choose_limit-2) {
        d = all_parity(n2.map);
        if(!parity(n2.map, d.gox, d.goy)){
            n2.real_cost++;
            n2.choose_x = d.gox;
            n2.choose_y = d.goy;
            n2.how_move.push_back('o');
            n2.how_choose.push_back(d);
            n2.parent = &n;
            n2.status = 'O';
            prin(n2);
            exit(0);
        }
    }
    else d = firstposition(n2.map);
//    d = all_parity(n2.map);
//    if(d.gox == -1) d = firstposition(n2.map);
    n2.real_cost++;
    n2.choose_x = d.gox;
    n2.choose_y = d.goy;
    n2.how_move.push_back('o');
    n2.how_choose.push_back(d);
    n2.parent = &n;
    n2.status = 'O';
    heuristic(n2);
}

void move_c2(Node &n, Node &n2){ //親ノードを1手選択した子ノードを作成 v2
    n2 = n;
    Ido d = {0,0,0};
    n2.chooses++;
    d = all_parity(n2.map);
    if(d.gox == -1) d = firstposition(n2.map);
    n2.real_cost++;
    n2.choose_x = d.gox;
    n2.choose_y = d.goy;
    n2.how_move.push_back('o');
    n2.how_choose.push_back(d);
    n2.parent = &n;
    n2.status = 'O';
    heuristic(n2);
}

void pair_swap(Node &n){
    int sizex = int(n.map[0].size()), sizey = int(n.map.size());
    Ido a = {0,0,0}, u, d, r, l;
    a = firstposition(n.map);
    u = {a.gox,a.goy-1,0};
    if(u.goy < 0) u.goy += sizey;
    d = {a.gox,a.goy+1,0};
    if(d.goy >= sizey) d.goy = 0;
    r = {a.gox+1,a.goy,0};
    if(r.gox >= sizex) r.gox = 0;
    l = {a.gox-1,a.goy,0};
    if(l.gox < 0) l.gox += sizex;
    
    if(n.map[a.goy][a.gox].gox == u.gox && n.map[a.goy][a.gox].goy == u.goy){
        n.real_cost += 2;
        n.chooses++;
        n.choose_x = a.gox;
        n.choose_y = a.goy;
        n.how_move.push_back('o');
        n.how_choose.push_back(a);
        n.how_move.push_back('U');
        U(n.map, n.choose_x, n.choose_y);
    }
    else if(n.map[a.goy][a.gox].gox == d.gox && n.map[a.goy][a.gox].goy == d.goy){
        n.real_cost += 2;
        n.chooses++;
        n.choose_x = a.gox;
        n.choose_y = a.goy;
        n.how_move.push_back('o');
        n.how_choose.push_back(a);
        n.how_move.push_back('D');
        D(n.map, n.choose_x, n.choose_y);
    }
    else if(n.map[a.goy][a.gox].gox == r.gox && n.map[a.goy][a.gox].goy == r.goy){
        n.real_cost += 2;
        n.chooses++;
        n.choose_x = a.gox;
        n.choose_y = a.goy;
        n.how_move.push_back('o');
        n.how_choose.push_back(a);
        n.how_move.push_back('R');
        R(n.map, n.choose_x, n.choose_y);
    }
    else if(n.map[a.goy][a.gox].gox == l.gox && n.map[a.goy][a.gox].goy == l.goy){
        n.real_cost += 2;
        n.chooses++;
        n.choose_x = a.gox;
        n.choose_y = a.goy;
        n.how_move.push_back('o');
        n.how_choose.push_back(a);
        n.how_move.push_back('L');
        L(n.map, n.choose_x, n.choose_y);
    }
    heuristic(n);
}

void T_U_X(Node &n){ //target up(x)
    n.real_cost += 5;
    n.how_move.push_back('U');
    n.how_move.push_back('L');
    n.how_move.push_back('D');
    n.how_move.push_back('R');
    n.how_move.push_back('U');
    U(n.map, n.choose_x, n.choose_y);
    L(n.map, n.choose_x, n.choose_y);
    D(n.map, n.choose_x, n.choose_y);
    R(n.map, n.choose_x, n.choose_y);
    U(n.map, n.choose_x, n.choose_y);
}

void T_D_X(Node &n){ //target down(x)
    n.real_cost += 5;
    n.how_move.push_back('D');
    n.how_move.push_back('L');
    n.how_move.push_back('U');
    n.how_move.push_back('R');
    n.how_move.push_back('D');
    D(n.map, n.choose_x, n.choose_y);
    L(n.map, n.choose_x, n.choose_y);
    U(n.map, n.choose_x, n.choose_y);
    R(n.map, n.choose_x, n.choose_y);
    D(n.map, n.choose_x, n.choose_y);
}

void T_L_X(Node &n){ //target left(x)
    n.real_cost += 5;
    n.how_move.push_back('D');
    n.how_move.push_back('L');
    n.how_move.push_back('L');
    n.how_move.push_back('U');
    n.how_move.push_back('R');
    D(n.map, n.choose_x, n.choose_y);
    L(n.map, n.choose_x, n.choose_y);
    L(n.map, n.choose_x, n.choose_y);
    U(n.map, n.choose_x, n.choose_y);
    R(n.map, n.choose_x, n.choose_y);
}

void T_U_Y(Node &n){ //target up(y)
    n.real_cost += 5;
    n.how_move.push_back('R');
    n.how_move.push_back('U');
    n.how_move.push_back('U');
    n.how_move.push_back('L');
    n.how_move.push_back('D');
    R(n.map, n.choose_x, n.choose_y);
    U(n.map, n.choose_x, n.choose_y);
    U(n.map, n.choose_x, n.choose_y);
    L(n.map, n.choose_x, n.choose_y);
    D(n.map, n.choose_x, n.choose_y);
}

void T_L_Y(Node &n){ //target left(y)
    n.real_cost += 5;
    n.how_move.push_back('L');
    n.how_move.push_back('U');
    n.how_move.push_back('R');
    n.how_move.push_back('D');
    n.how_move.push_back('L');
    L(n.map, n.choose_x, n.choose_y);
    U(n.map, n.choose_x, n.choose_y);
    R(n.map, n.choose_x, n.choose_y);
    D(n.map, n.choose_x, n.choose_y);
    L(n.map, n.choose_x, n.choose_y);
}

void T_R_Y(Node &n){ //target right(y)
    n.real_cost += 5;
    n.how_move.push_back('R');
    n.how_move.push_back('U');
    n.how_move.push_back('L');
    n.how_move.push_back('D');
    n.how_move.push_back('R');
    R(n.map, n.choose_x, n.choose_y);
    U(n.map, n.choose_x, n.choose_y);
    L(n.map, n.choose_x, n.choose_y);
    D(n.map, n.choose_x, n.choose_y);
    R(n.map, n.choose_x, n.choose_y);
}

void T_swap_X(Node &n){
    n.real_cost += 12;
    n.how_move.push_back('L');
    n.how_move.push_back('U');
    n.how_move.push_back('R');
    n.how_move.push_back('D');
    n.how_move.push_back('R');
    n.how_move.push_back('U');
    n.how_move.push_back('L');
    n.how_move.push_back('L');
    n.how_move.push_back('D');
    n.how_move.push_back('R');
    n.how_move.push_back('U');
    n.how_move.push_back('R');
    L(n.map, n.choose_x, n.choose_y);
    U(n.map, n.choose_x, n.choose_y);
    R(n.map, n.choose_x, n.choose_y);
    D(n.map, n.choose_x, n.choose_y);
    R(n.map, n.choose_x, n.choose_y);
    U(n.map, n.choose_x, n.choose_y);
    L(n.map, n.choose_x, n.choose_y);
    L(n.map, n.choose_x, n.choose_y);
    D(n.map, n.choose_x, n.choose_y);
    R(n.map, n.choose_x, n.choose_y);
    U(n.map, n.choose_x, n.choose_y);
    R(n.map, n.choose_x, n.choose_y);
}

void T_swap_Y(Node &n){
    n.real_cost += 12;
    n.how_move.push_back('U');
    n.how_move.push_back('L');
    n.how_move.push_back('D');
    n.how_move.push_back('R');
    n.how_move.push_back('D');
    n.how_move.push_back('L');
    n.how_move.push_back('U');
    n.how_move.push_back('U');
    n.how_move.push_back('R');
    n.how_move.push_back('D');
    n.how_move.push_back('L');
    n.how_move.push_back('D');
    U(n.map, n.choose_x, n.choose_y);
    L(n.map, n.choose_x, n.choose_y);
    D(n.map, n.choose_x, n.choose_y);
    R(n.map, n.choose_x, n.choose_y);
    D(n.map, n.choose_x, n.choose_y);
    L(n.map, n.choose_x, n.choose_y);
    U(n.map, n.choose_x, n.choose_y);
    U(n.map, n.choose_x, n.choose_y);
    R(n.map, n.choose_x, n.choose_y);
    D(n.map, n.choose_x, n.choose_y);
    L(n.map, n.choose_x, n.choose_y);
    D(n.map, n.choose_x, n.choose_y);
}

void carry_X(Node &n, int x, int y){
    if(n.choose_y != y){
        if(n.choose_y > y){
            T_U_X(n);
        }
        else {
            T_D_X(n);
        }
    }
    else if(n.choose_x > x){
        T_L_X(n);
    }
}

void carry_Y(Node &n, int x, int y){
    if(n.choose_x != x){
        if(n.choose_x > x){
            T_L_Y(n);
        }
        else {
            T_R_Y(n);
        }
    }
    else if(n.choose_y > y){
        T_U_Y(n);
    }
}

void go_T_X(Node &n, int x, int y){
    if(x == n.map[0].size()-1){
        while(n.choose_y != y){
            if(n.choose_y > y){
                n.real_cost++;
                n.how_move.push_back('U');
                U(n.map, n.choose_x, n.choose_y);
            }
            else {
                n.real_cost++;
                n.how_move.push_back('D');
                D(n.map, n.choose_x, n.choose_y);
            }
        }
    }
    if(n.choose_x <= x && n.choose_y == y)  while(n.choose_x < x) {
        n.real_cost++;
        n.how_move.push_back('R');
        R(n.map, n.choose_x, n.choose_y);
    }
    else{
        x++;
        while(n.choose_x != x){
            if(n.choose_x > x){
                n.real_cost++;
                n.how_move.push_back('L');
                L(n.map, n.choose_x, n.choose_y);
            }
            else {
                n.real_cost++;
                n.how_move.push_back('R');
                R(n.map, n.choose_x, n.choose_y);
            }
        }
        while(n.choose_y != y){
            if(n.choose_y > y){
                n.real_cost++;
                n.how_move.push_back('U');
                U(n.map, n.choose_x, n.choose_y);
            }
            else {
                n.real_cost++;
                n.how_move.push_back('D');
                D(n.map, n.choose_x, n.choose_y);
            }
        }
    }
}

void go_T_Y(Node &n, int x, int y){
    if(y == n.map.size()-1){
        while(n.choose_x != x){
            if(n.choose_x > x){
                n.real_cost++;
                n.how_move.push_back('L');
                L(n.map, n.choose_x, n.choose_y);
            }
            else {
                n.real_cost++;
                n.how_move.push_back('R');
                R(n.map, n.choose_x, n.choose_y);
            }
        }
    }
    if(n.choose_x == x && n.choose_y <= y)  while(n.choose_y < y) {
        n.real_cost++;
        n.how_move.push_back('D');
        D(n.map, n.choose_x, n.choose_y);
    }
    else{
        y++;
        while(n.choose_y != y){
            if(n.choose_y > y){
                n.real_cost++;
                n.how_move.push_back('U');
                U(n.map, n.choose_x, n.choose_y);
            }
            else {
                n.real_cost++;
                n.how_move.push_back('D');
                D(n.map, n.choose_x, n.choose_y);
            }
        }
        while(n.choose_x != x){
            if(n.choose_x > x){
                n.real_cost++;
                n.how_move.push_back('L');
                L(n.map, n.choose_x, n.choose_y);
            }
            else {
                n.real_cost++;
                n.how_move.push_back('R');
                R(n.map, n.choose_x, n.choose_y);
            }
        }
    }
}

#endif /* INCLUDE_GUARD_SOLVER2_FUNC */
