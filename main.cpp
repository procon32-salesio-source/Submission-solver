#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>
#include <cstdio>
#include <queue>
#include "solver3_func.hpp"
using namespace std;
#define rep(i, b) for(int i = 0; i < int(b); i++)

int main(int argc, const char * argv[]) {
    ifstream in("input.txt");
    ofstream out("solution.txt");
//    Ido a = {0,0,0};
    int gridy, gridx, choose_limit, cost_chos, cost_swap; //縦,横,選択可能回数,選択コスト,交換コスト
    in >> gridx >> gridy >> choose_limit >> cost_chos >> cost_swap; //を読み込む
    vector<vector<Ido>> mov(gridy, vector<Ido>(gridx));
    rep(i, gridy){
        rep(j, gridx){
            in >> mov[i][j].gox >> mov[i][j].goy >> mov[i][j].rot; //ファイルから読み込む
        }
    }
    
    
    /* 大量の変数たち */
    Ido first_ido = all_parity(mov); //初期位置の決定
    bool nite = 0; //一手で終わるかどうか 使ってない
    if(first_ido.gox == -1){
        nite = 1;
        first_ido = firstposition(mov);
    }
    cout << first_ido.gox << " " << first_ido.goy << " " << nite << endl;
    
    int beam_size = 300; //beam幅
    vector<char> goal_way(0); //not using
    Node nod, nod2; //Node型 盤面の情報や移動経路を保存
    init(nod, mov, first_ido.gox, first_ido.goy, choose_limit, cost_chos, cost_swap); //init
    init(nod2, mov, first_ido.gox, first_ido.goy, choose_limit, cost_chos, cost_swap); //init
    
    auto lamb = [](const Node& a, const Node& b){ return a.heur_cost > b.heur_cost; }; //lambda関数 priority_queueに必要
    priority_queue<Node, vector<Node>, decltype(lamb)> open_list(lamb), next_list(lamb), last_list(lamb); //beamの核
    
//    return 0;
    
    int ov_x, ov_y;
    Ido new_d = {0,0,0};
    ov_x = max(gridx-5, 0);
    ov_y = max(gridy-5, 0);
    if(ov_x > 0 && ov_y > 0){
        new_d = where(nod.map, gridx-1, gridy-1);
        nod.how_choose[0] = new_d;
        nod.choose_x = new_d.gox;
        nod.choose_y = new_d.goy;
        cout << new_d.gox << " " << new_d.goy << endl;
    }
    if(ov_x > 0){
        rep(i, ov_x){
            rep(j, gridy){
                if(j+2 < gridy){
                    new_d = where(nod.map, i, j);
                    go_T_X(nod, new_d.gox, new_d.goy);
                    while(manhattan(nod.map, i, j) != 0){
                        carry_X(nod, i, j);
                    }
                    //while end
                }
                else if(j+2 == gridy){
                    new_d = where(nod.map, i, j+1);
                    go_T_X(nod, new_d.gox, new_d.goy);
                    while(!(nod.map[j][i].gox == i && nod.map[j][i].goy == j+1)){
                        carry_X(nod, i, j);
                    }
                    j++;
                    new_d = where(nod.map, i, j-1);
                    go_T_X(nod, new_d.gox, new_d.goy);
                    if(nod.map[j][i].gox == i && nod.map[j][i].goy == j-1) T_swap_X(nod);
                    while(!(nod.map[j-1][i+1].gox == i && nod.map[j-1][i+1].goy == j-1)){
                        carry_X(nod, i+1, j-1);
                    }
                    T_L_X(nod);
                }
//                prin(nod);
                //carried target
            }
        }
        move(nod, 'R', nod);
//        prin(nod);
    }
    //if ov_x end
    
    if(ov_y > 0){
        rep(j, ov_y){
            for(int i = ov_x; i < gridx; i++){
                if(i+2 < gridx){
                    new_d = where(nod.map, i, j);
                    go_T_Y(nod, new_d.gox, new_d.goy);
                    while(manhattan(nod.map, i, j) != 0){
                        carry_Y(nod, i, j);
                    }
                    //while end
                }
                else if(i+2 == gridx){
                    new_d = where(nod.map, i+1, j);
                    go_T_Y(nod, new_d.gox, new_d.goy);
                    while(!(nod.map[j][i].gox == i+1 && nod.map[j][i].goy == j)){
                        carry_Y(nod, i, j);
                    }
                    i++;
                    new_d = where(nod.map, i-1, j);
                    go_T_Y(nod, new_d.gox, new_d.goy);
                    if(nod.map[j][i].gox == i-1 && nod.map[j][i].goy == j) T_swap_Y(nod);
                    while(!(nod.map[j+1][i-1].gox == i-1 && nod.map[j+1][i-1].goy == j)){
                        carry_Y(nod, i-1, j+1);
                    }
                    T_U_Y(nod);
                }
//                prin(nod);
                //carried target
            }
        }
        move(nod, 'D', nod);
//        prin(nod);
    }
    //if ov_y end
    
    nod.status = 'O';
    heuristic(nod);
    open_list.push(nod);
    int cnt = 0, siz = 0;
    
    while(nod.heur_cost > 6){
//        if(nod.heur_cost <= 9) cout << nod.heur_cost;
//        cout << nod.how_move[0];
        for(int i = 0; i < beam_size; i++){
            siz = int(nod.how_move.size());
            nod = open_list.top();
            open_list.pop();
            nod.status = 'C';
            if(nod.how_move[siz-1] != 'D') {
                move(nod, 'U', nod2);
                next_list.push(nod2);
            }
            if(nod.how_move[siz-1] != 'U') {
                move(nod, 'D', nod2);
                next_list.push(nod2);
            }
            if(nod.how_move[siz-1] != 'L') {
                move(nod, 'R', nod2);
                next_list.push(nod2);
            }
            if(nod.how_move[siz-1] != 'R') {
                move(nod, 'L', nod2);
                next_list.push(nod2);
            }
            if(manhattan(nod.map, nod.choose_x, nod.choose_y) == 0 && nod.chooses < nod.choose_limit-2 && mt() % 30 <= 0) {
                move_c(nod, nod2);
                next_list.push(nod2);
            }
            if(manhattan(nod.map, nod.choose_x, nod.choose_y) == 0 && nod.chooses == nod.choose_limit-3 && nod.heur_cost <= 18) {
                move_c2(nod, nod2);
                next_list.push(nod2);
            }
            if(isGoal(nod.map)) break;
            if(nod.heur_cost <= 6) {
                last_list.push(nod);
                if(last_list.size() == 10){
                    prin(nod);
                    break;
                }
            }
            if(open_list.empty()) {
                cout << "x";
                break;
            }
        }
        cnt++;
        if(cnt % 50 == 0) {
            cout << ".";
            if(nod.chooses == nod.choose_limit){
                cout << nod.how_choose[9].gox << nod.how_choose[9].goy << " " << parity(nod.map, nod.how_choose[9].gox, nod.how_choose[9].goy) << endl;
            }
        }
//        if(cnt % 200 == 0) prin(nod);
        if(cnt >= 10000){
            cout << "a" << endl;
            break;
        }
        
        while(!open_list.empty()) open_list.pop();
        for(int i = 0; i < beam_size+1; i++) {
            if(next_list.empty()) break;
            open_list.push(next_list.top());
            next_list.pop();
        }
        while(!next_list.empty()) next_list.pop();
    }
    
    while(!open_list.empty()) open_list.pop();
    for(int i = 0; i < 15; i++) {
        if(last_list.empty()) break;
        open_list.push(last_list.top());
        last_list.pop();
    }
    while(!next_list.empty()) next_list.pop();
    
    prin(nod);
    cout << "\n----\n\n";
    beam_size = 5000;
    int cnt2 = 0;
    while(!isGoal(nod.map) && !open_list.empty()){
        nod = open_list.top();
        open_list.pop();
        siz = int(nod.how_move.size());
        
        if(isGoal(nod.map)) break;
        pair_swap(nod);
        if(isGoal(nod.map)) break;
        pair_swap(nod);
        if(isGoal(nod.map)) break;
        pair_swap(nod);
        
        if(open_list.empty()) {
            cout << "x";
            break;
        }
    }
    
    if(open_list.empty()) open_list.push(nod);
    while(!isGoal(nod.map)){
//        if(nod.heur_cost <= 9) cout << nod.heur_cost;
//        cout << nod.how_move[0];
        for(int i = 0; i < beam_size; i++){
            nod = open_list.top();
            open_list.pop();
            siz = int(nod.how_move.size());
            nod.status = 'C';
            if(nod.how_move[siz-1] != 'D') {
                move(nod, 'U', nod2);
                next_list.push(nod2);
            }
            if(nod.how_move[siz-1] != 'U') {
                move(nod, 'D', nod2);
                next_list.push(nod2);
            }
            if(nod.how_move[siz-1] != 'L') {
                move(nod, 'R', nod2);
                next_list.push(nod2);
            }
            if(nod.how_move[siz-1] != 'R') {
                move(nod, 'L', nod2);
                next_list.push(nod2);
            }
            if(nod.how_move[siz-1] != 'o') {
                move_c2(nod, nod2);
                next_list.push(nod2);
            }
            if(isGoal(nod.map)) break;
            if(open_list.empty()) {
                cout << "x";
                break;
            }
        }
        cnt++;
        cnt2++;
        if(cnt % 10 == 0) {
            cout << ".";
            if(nod.chooses == nod.choose_limit){
                cout << nod.how_choose[9].gox << nod.how_choose[9].goy << " " << parity(nod.map, nod.how_choose[9].gox, nod.how_choose[9].goy) << endl;
            }
        }
//        if(cnt % 40 == 0) prin(nod);
        if(cnt >= 4000){
            cout << "a" << endl;
            break;
        }
        
        while(!open_list.empty()) open_list.pop();
        for(int i = 0; i < beam_size+1; i++) {
            if(next_list.empty()) break;
            open_list.push(next_list.top());
            next_list.pop();
        }
        while(!next_list.empty()) next_list.pop();
    }
    
    cout << cnt << endl;
    prin(nod);
    if(isGoal(nod.map)){
        int scores = 0;
        scores += int(nod.how_move.size() - nod.chooses) * cost_swap;
        scores += nod.chooses * cost_chos;
        cout << "Total score : " << scores << endl;
        if(nod.chooses > choose_limit) cout << "Not ";
        cout << "Happy " << endl;
    }
    cout << endl;
    
    vector<int> r1(gridx*gridy);
    rep(j, gridy) rep(i, gridx) r1[nod.map[j][i].gox + nod.map[j][i].goy * gridx] = nod.map[j][i].rot;
    rep(i, gridx*gridy) out << r1[i];
    out << endl << nod.chooses << endl;
    
    vector<Line> kaito(nod.chooses);
    int chose = -1;
    rep(i, nod.how_move.size()){
        if(nod.how_move[i] == 'o'){
            chose++;
            kaito[chose].pozx = nod.how_choose[chose].gox;
            kaito[chose].pozy = nod.how_choose[chose].goy;
            kaito[chose].sosa = "";
        }
        else {
            kaito[chose].sosa += nod.how_move[i];
        }
    }
    rep(i, nod.chooses){
        out << hx(kaito[i].pozx) << hx(kaito[i].pozy) << endl;
        out << kaito[i].sosa.size() << endl;
        out << kaito[i].sosa << endl;
    }
    
    return 0;
}
