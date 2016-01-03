#include "HPCAnswer.hpp"
#include "HPCMath.hpp"
#include "vector"
#include "queue"
#include "stack"
#include "iostream"
#include "bitset"
#include "algorithm"
#include "unordered_map"

#define FOR(i,a,b) for(int i=(a);i<int(b);++i)
#define REP(i,n)  FOR(i,0,n)
#define inf 715827882 // INT_MAX/3

#define RANDAMLOOP 14000 // 提出時に変更するループ回数　16385　6385


using namespace std;



namespace hpc {
    int stage_n;                // ステージナンバー
    int successd_stage_n;
    int failed_stage_n;
    int total_score;
    int n_score;
    vector<queue<Action>> rAct;  // rAct[時間帯] = アクションリスト　nStage.act をコピーする
    vector<vector<int>> rBag;    // rBag[時間帯] = 荷物リスト        nStage.bag をコピーする
    const int x[]={-1,1,0,0};    // 上下左右のActionの選択用
    const int y[]={0,0,-1,+1};
    const Action actionList[]={Action_MoveLeft,Action_MoveRight,Action_MoveDown,Action_MoveUp};
    const Action actionListReversed[]={Action_MoveRight,Action_MoveLeft,Action_MoveUp,Action_MoveDown};
    
    // 毎ステージで使うクラス　aStageの混合しないように注意
    class nStage{
    public:
        nStage();                                       // 初期設定 nStage::nStage()
        const Stage *aStage;                            // nStage内でaStageを参照するためのもの
        void getStage(const Stage& aStageSub);
        void solve();
        void ForceSolve();
    private:
        vector<vector<vector<int>>> allMap;  // allMap[基準点][y][x] = Pos(基準点)から(x,y)までの距離
        vector<vector<int>> bag;             // bag[時間帯] = 荷物リスト
        vector<queue<Action>> act;           // act[時間帯] = アクションリスト
        unordered_map <string, string>mp;
        int mid_x,mid_y;                                 // 中心点
        void SetAllMap();                                // allMapを埋める
        void SetBag();                                   // 荷物リストからbag[時間帯]に入れる
        void SetAction();                                // 配送リストからアクションを埋める
        void SetActionStart2A(int time,int p);           // 中心点からPos(p)までのアクションを埋める > nStage.act
        void SetActionA2B(int time,int p, int n, int m); // Pos(p)から(n,m)までのアクションを埋める > nStage.act
        void SetActionB2End(int time,int p);             // Pos(p)から(中心点までのアクションを埋める　> nStage.act
        void CopyGlovalANS();                            // rAct, rBag にコピーする
        void Greedy();                                   // bag[-1]の振り分けの全探索
        void ForceForce();
        void ForceRoot(int t);                           // ルート決定　全探索 bag[t]を指定する
        void GreedyRoot();                               // ルート決定　貪欲法
        void GreedyRootT(int t);                         // ルート決定　貪欲法 bag(t)
        void Opt2(int t);                                // ルート改善  2_Opt法(距離)
        void Opt2Fuel(int t);                            // ルート改善  2_Opt法(燃料)
        void ReverseRoot(int t);                         // ルート改善　逆順を試す(消費燃料を見る)
        int FuelCostR(vector<int> root);                 // ルートの消費燃料
        void ReplaceBag();                               // 荷物を他のところににうつす
        void ReplaceBagFuel();                               // 荷物を他のところににうつす+Fuel
        void ExchangeBag();                              // 荷物を他の荷物と交換する
        void ExchangeBagFuel();                              // 荷物を他の荷物と交換する+Fuel
        int DistanceAB(int a_index, int itemB_index);    // 点A,点Bとの距離
        int DistanceAxy(int a_index, int x, int y);      // 点A,点(x,y)との距離
        int BagWeight(vector<int> b);                    // バッグの重さ
        int ItemWeightBagIndex(int time,int bag_index);  // 重さ
        int ItemWeight(int item_index);                  // 重さ2
        bool overWeight();                               // 荷物が15を超えているか
        long long Pow(int x, int y);                     // power
        void StartBagChange();                           // 荷物(-1)が多い時、初期のバッグをいじっておく
        void StageScore();
        string Root2Hash(vector<int> root);
    };
    
    nStage::nStage()
    :allMap(20,vector<vector<int>>(100,vector<int>(100))),bag(5),act(4)
    {}
    
    void nStage::getStage(const Stage& aStageSub){
        aStage=&aStageSub;
        mid_x = aStage->field().width()/2;
        mid_y = aStage->field().height()/2;
    }
    
    void nStage::solve(){
        SetBag();
        SetAllMap();
        if (bag[4].size()<4) {
            // 5にするとスコアが下がるがバグが解決しない
//            printf("貪欲法");
            ForceForce();
        }else{
            if (bag[4].size()>14 && bag[0].size()<=1 && bag[1].size()<=1 && bag[2].size()<=1 && bag[3].size()<=1) {
//                StartBagChange();
            }
            Greedy();
            ReplaceBagFuel();
            ExchangeBagFuel();
            REP(t, 4){
                Opt2Fuel(t);
            }
            ReplaceBagFuel();
            ExchangeBagFuel();
            REP(t, 4){
                Opt2Fuel(t);
            }
            REP(t, 4){
                ForceRoot(t);
            }
        }
        SetAction();
        CopyGlovalANS();
        StageScore();
    }
    
    
    // 全探索でどこまでいけるか
    void nStage::ForceSolve(){
        SetBag();
        SetAllMap();
        if (bag[4].size()==0) {
            REP(t, 4){
                ForceRoot(t);
            }
        }else{
            if(bag[4].size()<8){
                ForceForce();
            }
        }
        SetAction();
        CopyGlovalANS();
        StageScore();
    }
    
    void nStage::StageScore(){
        int fuel = 0;
        REP(t, 4){
            fuel += FuelCostR(bag[t]);
        }
        if (fuel==0) {
            return;
        }
        n_score = aStage->field().width() * aStage->field().height() * aStage->items().count() *10000/fuel;
//        printf("%d",n_score);
    }
    
    
    
    void nStage::Greedy(){
        vector<vector<int>> init_bag(4);
        vector<vector<int>> max_bag(4);
        for (int i = 0; i<4; ++i) {
            copy(bag[i].begin(),bag[i].end(),back_inserter(init_bag[i]));
        }
        int min_fuel = inf;
        vector<int> delivery_time;
        for (int q=0; q<RANDAMLOOP; ++q) {
            //   bagを初期値に戻す
            REP(t, 4){
                bag[t].clear();
                copy(init_bag[t].begin(), init_bag[t].end(), back_inserter(bag[t]));
            }
            // 数字に基づいて　配達時間を決定
            delivery_time.clear();
            for (int j=0; j<int(bag[4].size()); ++j) {
                delivery_time.push_back(rand()%4);
            }
            // 指定した時間のバッグに詰める
            for (int j = 0; j<int(bag[4].size()); ++j) {
                bag[delivery_time[j]].push_back(bag[4][j]);
            }
            // 荷物が積載可能重量を超えていたらパス
            if (overWeight()) {
                continue;
            }
            // 回数が多いのでルートぎめは貪欲法でとく
            GreedyRoot();
//            ReplaceBagFuel();
//            ExchangeBagFuel();
//            REP(t, 4){
//                Opt2Fuel(t);
//            }
            // 消費燃料の計算
            int total_fuel = 0;
            for (int t=0; t<4; ++t) {
                total_fuel += FuelCostR(bag[t]);
            }
            if (total_fuel<min_fuel) {
                min_fuel = total_fuel;
                for (int k=0; k<4; ++k) {
                    max_bag[k].clear();
                    copy(bag[k].begin(),bag[k].end(),back_inserter(max_bag[k]));
                }
            }
        }
        //// ここまで　ランダムループ
        /// max_bagをbagにコピー
        for (int t=0; t<4; ++t) {
            bag[t].clear();
            copy(max_bag[t].begin(),max_bag[t].end(),back_inserter(bag[t]));
        }
    }
    
    
    // 全探索x全探索 Greedyのコピペ
    void nStage::ForceForce(){
        vector<vector<int>> init_bag(5);
        vector<vector<int>> max_bag(4);
        for (int i = 0; i<5; ++i) {
            copy(bag[i].begin(),bag[i].end(),back_inserter(init_bag[i]));
        }
        int min_fuel = inf;
        vector<int> delivery_time;
        for (long long i=0; i<Pow(4, int(bag[4].size())); ++i) {
            //   bagを初期値に戻す
            for (int j = 0; j<4; ++j) {
                bag[j].clear();
                copy(init_bag[j].begin(),init_bag[j].end(),back_inserter(bag[j]));   //よくわからないエラー
            }
            // 数字に基づいて　配達時間を決定
            delivery_time.clear();
            long long ii = i;
            for (int j=0; j<int(bag[4].size()); ++j) {
                delivery_time.push_back(ii&3);
                ii>>=2;
            }
            //
//                        REP(i, delivery_time.size()){
//                            printf("%d ",delivery_time[i]);
//                        }
//                        printf("\n");
            //
            
            // 指定した時間のバッグに詰める
            for (int j = 0; j<int(bag[4].size()); ++j) {
                bag[delivery_time[j]].push_back(bag[4][j]);
            }
            // 荷物が積載可能重量を超えていたらパス
            if (overWeight()) {
                continue;
            }
            REP(t, 4){ForceRoot(t);}
            // 消費燃料の計算
            int total_fuel = 0;
            for (int t=0; t<4; ++t) {
                total_fuel += FuelCostR(bag[t]);
            }
            if (total_fuel<min_fuel) {
                min_fuel = total_fuel;
                REP(k, 4){
                    max_bag[k].clear();
                    copy(bag[k].begin(), bag[k].end(), back_inserter(max_bag[k]));
                }
            }
        }
        /// max_bagをbagにコピー
        for (int t=0; t<4; ++t) {
            bag[t].clear();
            copy(max_bag[t].begin(),max_bag[t].end(),back_inserter(bag[t]));
        }
    }
    
    
    
    
    void nStage::ReplaceBag(){
        vector<int> can_replace(20);
        REP(i, bag[4].size()){
            can_replace[bag[4][i]] = 1;
        }
        ///// ループするならここ
        int count; // ループしても改善点がみつからないとき(count==0)にループを抜ける
        while (true) {
            count = 0;
            REP(t,4){
                REP(i, bag[t].size()){ // ひとつえらぶ
                    if (can_replace[bag[t][i]]!=1) {
                        continue;
                    }
                    int d1 = 0;
                    int a1,b1; // 両端のindex
                    if (i==0) {
                        d1 -= DistanceAB(bag[t][i], 99);
                        a1 = 99;
                    }else{
                        d1 -= DistanceAB(bag[t][i], bag[t][i-1]);
                        a1 = bag[t][i-1];
                    }
                    if (i==int(bag[t].size()-1)) {
                        d1 -= DistanceAB(int(bag[t][i]), 99);
                        b1 =99;
                    }else{
                        d1 -= DistanceAB(int(bag[t][i]), bag[t][i+1]);
                        b1 = bag[t][i+1];
                    }
                    d1 += DistanceAB(a1, b1);
                    // 挿入先の探索
                    int min_d2 = inf;
                    int d2 = 0;
                    int a2=0;
                    int b2=0;
                    int min_t2 = 0;
                    int min_j = 0;
                    int flag_i = 0;
                    REP(t2, 4){
                        if (t==t2) {
                            continue;
                        }
                        // insertに最適な場所を探す
                        if (ItemWeight(bag[t][i])+BagWeight(bag[t2])>=15) {
                            continue;
                        }
                        REP(j, bag[t2].size()){
                            d2 = 0;
                            if (j==0) {
                                d2 += DistanceAB(99, bag[t][i]);
                                a2 = 99;
                            }else{
                                d2 += DistanceAB(bag[t2][j-1], bag[t][i]);
                                a2 = bag[t2][j-1];
                            }
                            if (j==int(bag[t2].size())) {
                                d2 += DistanceAB(bag[t][i], 99);
                                b2 = 99;
                            }else{
                                d2 += DistanceAB(bag[t2][j],bag[t][i]);
                                b2 = bag[t2][j];
                            }
                            d2 -= DistanceAB(a2, b2);
                            //                            assert(d2>=0);
                            if (d2 < min_d2) {
                                min_d2 = d2;
                                min_j = j;
                                min_t2 = t2;
                                flag_i = 1;
                                //                            break;
                            }
                        }
                    }
                    //                    assert(d1<=0);
                    if (d1+min_d2<-20 && flag_i==1) {
                        bag[min_t2].insert(bag[min_t2].begin()+min_j, bag[t][i]);
                        bag[t].erase(bag[t].begin()+i);
                        count ++;
                        flag_i = 0;
                    }
                }
            }
            if (count==0) {
                break;
            }
        }
    }
    
    void nStage::ReplaceBagFuel(){
        vector<int> can_replace(20);
        vector<vector<int>> tmp_bag(4);
        REP(i, bag[4].size()){
            can_replace[bag[4][i]] = 1;
        }
        REP(i, 4){
            copy(bag[i].begin(), bag[i].end(), back_inserter(tmp_bag[i]));
        }
        ///// ループするならここ
        int count; // ループしても改善点がみつからないとき(count==0)にループを抜ける
        while (true) {
            int flag = 0;
            count = 0;
            REP(t,4){
                if (bag[t].size()==0) {
                    continue;
                }
                if (flag==1) {
                    break;
                }
                REP(i, bag[t].size()){ // ひとつえらぶ
                    if (flag==1) {
                        break;
                    }
                    if (can_replace[bag[t][i]]!=1) {
                        continue;
                    }
                    // 挿入先の探索
                    REP(t2, 4){
                        if (flag==1) {
                            break;
                        }
                        if (t==t2) {
                            continue;
                        }
                        // insertに最適な場所を探す
                        if (ItemWeight(bag[t][i])+BagWeight(bag[t2])>15) {
                            continue;
                        }
                        REP(j, bag[t2].size()){
                            if (flag==1) {
                                break;
                            }
                            // コスト計算
                            int old_fuel_cost = 0;
                            REP(k, 4){
                                old_fuel_cost+=FuelCostR(bag[k]);
                            }
                            // 移動
                            bag[t2].insert(bag[t2].begin()+j, bag[t][i]);
                            bag[t].erase(bag[t].begin()+i);
                            // コスト計算2
                            int new_fuel_cost = 0;
                            REP(k, 4){
                                new_fuel_cost += FuelCostR(bag[k]);
                            }
                            // 更新できるか
                            if (new_fuel_cost<old_fuel_cost) {
//                                printf("%d\n",new_fuel_cost);
                                REP(l, 4){
                                    tmp_bag[l].clear();
                                    copy(bag[l].begin(),bag[l].end(),back_inserter(tmp_bag[l]));
                                }
                                count++;
                                flag = 1;
                            }else{
                                REP(l, 4){
                                    bag[l].clear();
                                    copy(tmp_bag[l].begin(),tmp_bag[l].end(),back_inserter(bag[l]));
                                }
                            }
                        }
                    }
                }
            }
            if (count==0) {
                break;
            }
        }
    }
    
    void nStage::ExchangeBag(){
        vector<int> baglist(bag[4].size());     // index = bag[4]_index
        vector<int> can_replace(20);
        REP(i, bag[4].size()){
            can_replace[bag[4][i]] = 1;
        }
        int count; // ループしても改善点がみつからないとき(count==0)にループを抜ける
        while (true) {
            count = 0;
            REP(t,4){
                REP(i, bag[t].size()){ // ひとつえらぶ
                    if (can_replace[bag[t][i]]!=1) {
                        continue;
                    }
                    REP(t2, 4){
                        REP(j, bag[t2].size()){
                            if (can_replace[bag[t2][j]]!=1) {
                                continue;
                            }
                            if (ItemWeight(bag[t][i])+BagWeight(bag[t2])-ItemWeight(bag[t2][j]>15) or
                                ItemWeight(bag[t2][j])+BagWeight(bag[t])-ItemWeight(bag[t][i])>15) {
                                continue;
                            }
                            int pre_d1 = 0;
                            int pre_d2 = 0;
                            int new_d1 = 0;
                            int new_d2 = 0;
                            if (i==0) {
                                pre_d1 += DistanceAB(99, bag[t][i]);
                                new_d1 += DistanceAB(99, bag[t2][j]);
                            }else{
                                pre_d1 += DistanceAB(bag[t][i-1], bag[t][i]);
                                new_d1 += DistanceAB(bag[t][i-1], bag[t2][j]);
                            }
                            if (i==int(bag[t].size())) {
                                pre_d1 += DistanceAB(bag[t][i], 99);
                                new_d1 += DistanceAB(bag[t2][j], 99);
                            }else{
                                pre_d1 += DistanceAB(bag[t][i], bag[t][i+1]);
                                new_d1  += DistanceAB(bag[t2][j], bag[t][i+1]);
                            }
                            if (j==0) {
                                pre_d2 += DistanceAB(99, bag[t2][j]);
                                new_d2 += DistanceAB(99, bag[t][i]);
                            }else{
                                pre_d2 += DistanceAB(bag[t2][j-1], bag[t2][j]);
                                new_d2 += DistanceAB(bag[t2][j-1], bag[t][i]);
                            }
                            if (j==int(bag[t2].size())) {
                                pre_d2 += DistanceAB(bag[t2][j], 99);
                                new_d2 += DistanceAB(bag[t][i], 99);
                            }else{
                                pre_d2 += DistanceAB(bag[t2][j], bag[t2][j+1]);
                                new_d2 += DistanceAB(bag[t][i], bag[t2][j+1]);
                            }
                            if ((pre_d1+pre_d1-new_d1-new_d2)>10) {
                                swap(bag[t][i], bag[t2][j]);
                            }
                        }
                    }
                }
            }
            if (count==0) {
                break;
            }
        }
    }
    
    void nStage::ExchangeBagFuel(){
        vector<int> baglist(bag[4].size());     // index = bag[4]_index
        vector<int> can_replace(20);
        REP(i, bag[4].size()){
            can_replace[bag[4][i]] = 1;
        }
        int count; // ループしても改善点がみつからないとき(count==0)にループを抜ける
        while (true) {
            count = 0;
            REP(t,4){
                REP(i, bag[t].size()){ // ひとつえらぶ
                    if (can_replace[bag[t][i]]!=1) {
                        continue;
                    }
                    REP(t2, 4){
                        REP(j, bag[t2].size()){
                            if (can_replace[bag[t2][j]]!=1) {
                                continue;
                            }
                            if (BagWeight(bag[t2])+ItemWeight(bag[t][i])-ItemWeight(bag[t2][j]>15) ||
                                BagWeight(bag[t])+ItemWeight(bag[t2][j])-ItemWeight(bag[t][i])>15) {
                                continue;
                            }
                            int old_fuel_cost = 0;
                            int new_fuel_cost = 0;
                            REP(k, 4){
                                old_fuel_cost += FuelCostR(bag[k]);
                            }
                            swap(bag[t][i],bag[t2][j]);
                            REP(k, 4){
                                new_fuel_cost += FuelCostR(bag[k]);
                            }
                            // 悪化したら戻す
                            if (new_fuel_cost>old_fuel_cost) {
                                swap(bag[t][i], bag[t2][j]);
                            }
                        }
                    }
                }
            }
            if (count==0) {break;}
        }
    }
    
    
    //    rootから消費燃料
    int nStage::FuelCostR(vector<int> root){
        if (root.size()==0) {
            return 0;
        }
        int truck_weight = 3;
        for (int i=0; i<int(root.size()); ++i) {
            truck_weight += ItemWeight(root[i]);
        }
                assert(truck_weight<=18);
        int fuel_consumption = 0;
        fuel_consumption += (truck_weight*DistanceAxy(root[0], mid_x, mid_y));
        truck_weight -= ItemWeight(root[0]);
        for (int i=0; i<int(root.size()-1); ++i) {
            fuel_consumption += (truck_weight*DistanceAB(root[i], root[i+1]));
            truck_weight -= ItemWeight(root[i+1]);
        }
                assert(truck_weight==3);
        fuel_consumption += truck_weight*DistanceAxy(root[int(root.size()-1)], mid_x, mid_y);
        return fuel_consumption;
    }
    
    // ルート探索の貪欲法
    // 方針　中心点(mid_x,midy)から最も近いものを選んでいく
    void nStage::GreedyRoot(){
        for (int t=0; t<4; ++t) {
            GreedyRootT(t);
        }
    }
    
    
    // ルート探索の貪欲法
    // 方針　中心点(mid_x,midy)から最も近いものを選んでいく
    void nStage::GreedyRootT(int t){
        // 荷物が1つの時は処理しない
        if (bag[t].size()<=1) {
            return;
        }
        // 荷物が全探索できる個数なら全探索
        // 個々に処理
        if (bag[t].size()<=5) {
            ForceRoot(t);
            return;
        }
        //
        int min_bag_index;
        for (int i=0; i<int(bag[t].size()); ++i) {
            // i==0のときは中心点に近いものを探す
            min_bag_index = i;
            int min_distance=inf;
            for (int j=i; j<int(bag[t].size());++j) {
                int d = 0;
                if (i==0) {
                    d = DistanceAxy(j, mid_x, mid_y);
                }else{
                    d = DistanceAB(i-1, j);
                }
                if (d<min_distance) {
                    min_distance = d;
                    min_bag_index = j;
                }
            }
            swap(bag[t][i],bag[t][min_bag_index]);
        }
        Opt2(t);
//        Opt2Fuel(t);
        ReverseRoot(t);
        return;
    }
    
    // ルート改善　2_Opt法
    // 参考　http://www.geocities.jp/m_hiroi/light/pyalgo64.html
    void nStage::Opt2(int t){
        while (true) {
            int count = 0;
            for (int i = 0; i<int(bag[t].size()-2); ++i) {
                int i1 = i + 1;
                for (int j=i+2; j<int(bag[t].size()); ++j) {
                    int j1;
                    if (j==int(bag[t].size())-1) {
                        j1 = 0;
                    }else{
                        j1 = j+1;
                    }
                    int l1,l2,l3,l4;
                    if (i!=0 || j1!=0) {
                        l1 = DistanceAB(bag[t][i], bag[t][i1]);
                        l2 = DistanceAB(bag[t][j], bag[t][j1]);
                        l3 = DistanceAB(bag[t][i], bag[t][j]);
                        l4 = DistanceAB(bag[t][i1], bag[t][j1]);
                        if (l1+l2>l3+l4) {
                            reverse(bag[t].begin(), bag[t].end());
                            reverse(bag[t].end()-j-1, bag[t].end()-i1);
                            count += 1;
                        }
                    }
                }
            }
            if (count == 0) {
                break;
            }
        }
    }
    
    
    void nStage::Opt2Fuel(int t){
        vector<int> root;
        while (true) {
            int count = 0;
            for (int i = 0; i<int(bag[t].size()-2); ++i) {
                int i1 = i + 1;
                for (int j=i+2; j<int(bag[t].size()); ++j) {
                    int j1;
                    if (j==int(bag[t].size())-1) {
                        j1 = 0;
                    }else{
                        j1 = j+1;
                    }
                    root.clear();
                    copy(bag[t].begin(),bag[t].end(),back_inserter(root));
                    reverse(root.begin(), root.end());
                    reverse(root.end()-j-1, root.end()-i1);
                    if (i!=0 || j1!=0) {
//                        printf("▲");
                        if (FuelCostR(bag[t])>FuelCostR(root)) {
                            bag[t].clear();
                            copy(root.begin(),root.end(),back_inserter(bag[t]));
                            count += 1;
//                            printf("○");
                        }else{
//                            printf("☓");
                        }
                    }
                }
            }
            if (count == 0) {
                break;
            }
        }
    }
    
    // ルート改善　逆順のチェック
    void nStage::ReverseRoot(int t){
        vector<int> root;
        copy(bag[t].begin(),bag[t].end(),back_inserter(root));
        reverse(root.begin(),root.end());
        if (FuelCostR(bag[t])>FuelCostR(root)) {
            reverse(bag[t].begin(),bag[t].end());
        }
        return;
    }
    
    
    // ルート全探索
    void nStage::ForceRoot(int time){
        if(bag[time].size()<=1){return;}
        vector<int> tmp_bag;
        copy(bag[time].begin(), bag[time].end(), back_inserter(tmp_bag));
        sort(tmp_bag.begin(),tmp_bag.end());
        string sorted_key = Root2Hash(tmp_bag);
        if(mp.count(sorted_key) != 0) {
            // ハッシュが設定されている場合の処理
            string abcd_best = mp[sorted_key];
            for (int j=0; j<int(bag[time].size()); ++j) {
                bag[time][j] = abcd_best.at(j)-'a';
            }
            return;
        } else {
            // keyが設定されてない時
            vector<int> min_bag;
            int min_fuel = inf;
            do{
                int s_fuel = FuelCostR(tmp_bag);
//                REP(i, tmp_bag.size()){
//                    printf("%d ",tmp_bag[i]);
//                }
//                printf("\n");
                if (min_fuel>s_fuel){
                    min_fuel = s_fuel;
                    min_bag.clear();
                    copy(tmp_bag.begin(), tmp_bag.end(), back_inserter(min_bag));
                }
            }while (next_permutation(tmp_bag.begin(),tmp_bag.end()));
            string best_key = Root2Hash(min_bag);
            bag[time].clear();
            copy(min_bag.begin(),min_bag.end(),back_inserter(bag[time]));
            mp[sorted_key] = best_key;
        }
    }
    
    string nStage::Root2Hash(vector<int> root){
        string hash_key = "";
        REP(i, root.size()){
            hash_key.push_back(root[i]+'a');
        }
        return hash_key;
    }
    
    long long nStage::Pow(int _x, int _y){
        long long r = 1;
        for (int i=0; i<_y; ++i) {
            r *= _x;
        }
        return r;
    }
    
    //   初期化　bag に荷物をつめる　-1はbag[4]に
    void nStage::SetBag(){
        for (int i = 0; i < aStage->items().count(); ++i) {
            if (aStage->items().operator[](i).period()==-1) { // 時間未指定[-1]はbag[4]にいれる
                bag[4].push_back(i);
            }else{
                bag[aStage->items().operator[](i).period()].push_back(i);
            }
        }
    }
    
    
    //    bagの中の荷物の順番で回るためのActionをつめる
    void nStage::SetAction(){
        for (int i =0; i<4; ++i) {
            if (bag[i].size()>0) {
                SetActionStart2A(i, bag[i][0]);
            }
            if (int(bag[i].size())>1) {
                for (int j = 0; j<int(bag[i].size()-1); ++j) {
                    SetActionA2B(i, bag[i][j+1], aStage->items().operator[](bag[i][j]).destination().x,
                                 aStage->items().operator[](bag[i][j]).destination().y);
                }
            }
            if (bag[i].size()>0) {
                SetActionB2End(i, bag[i][bag[i].size()-1]);
            }
        }
    }
    
    //    (n,m)地点からpへの道順をact[time]につめる
    void nStage::SetActionA2B(int time,int p, int n, int m){
        if (bag[time].size()==0) {
            return;
        }
        int n_x = n;
        int n_y = m;
        int step = allMap[p][m][n];
        while (step>0) {
            for (int i=0; i<4; ++i) {
                if (allMap[p][n_y+y[i]][n_x+x[i]]==step-1 && aStage->field().isWall(n_x+x[i],n_y+y[i])==false) {
                    n_x += x[i];
                    n_y += y[i];
                    step = allMap[p][n_y][n_x];
                    act[time].push(actionList[i]);
                    break;
                }
            }
        }
    }
    
    void nStage::SetActionStart2A(int time,int p){
        if (bag[time].size()==0) {
            return;
        }
        int n_x = mid_x;
        int n_y = mid_y;
        int step = allMap[p][n_y][n_x];
        while (step>0) {
            for (int i=0; i<4; ++i) {
                if (allMap[p][n_y+y[i]][n_x+x[i]]==step-1 && aStage->field().isWall(n_x+x[i],n_y+y[i])==false) {
                    n_x += x[i];
                    n_y += y[i];
                    step = allMap[p][n_y][n_x];
                    act[time].push(actionList[i]);
                    break;
                }
            }
        }
    }
    
    void nStage::SetActionB2End(int time,int p){
        if (bag[time].size()==0) {
            return;
        }
        int n_x = aStage->field().width()/2;
        int n_y = aStage->field().height()/2;
        int step = allMap[p][n_y][n_x];
        stack<Action> s;
        while (step>0) {
            for (int i=0; i<4; ++i) {
                if (allMap[p][n_y+y[i]][n_x+x[i]]==step-1 && aStage->field().isWall(n_x+x[i],n_y+y[i])==false) {
                    n_x = n_x+x[i];
                    n_y = n_y+y[i];
                    step = allMap[p][n_y][n_x];
                    s.push(actionListReversed[i]);
                    break;
                }
            }
        }
        while (s.empty()==false) {
            act[time].push(s.top());
            s.pop();
        }
    }
    
    void nStage::SetAllMap(){
        queue<Pos> q;
        for (int z=0; z<aStage->items().count(); ++z) {
            int start_x =aStage->items().operator[](z).destination().x;
            int start_y =aStage->items().operator[](z).destination().y;
            q.push(Pos(start_x,start_y));
            while (q.empty()==false) {
                for (int i=0; i<4; ++i) {
                    int n_x = q.front().x+x[i];
                    int n_y = q.front().y+y[i];
                    if (n_x==start_x && n_y==start_y) {
                        continue;
                    }
                    if (aStage->field().isWall(n_x, n_y)==false){
                        if (allMap[z][n_y][n_x]==0) {
                            q.push(Pos(n_x,n_y));
                            allMap[z][n_y][n_x] = allMap[z][q.front().y][q.front().x]+1;
                        }
                    }
                }
                q.pop();
            }
        }
    }
    
    void nStage::CopyGlovalANS(){
        rAct = act;
        rBag = bag;
    }
    
    
    int nStage::ItemWeightBagIndex(int time,int item_index){
        return ItemWeight(bag[time][item_index]);
    }
    
    int nStage::ItemWeight(int id){
        return aStage->items().operator[](id).weight();
    }
    
    int nStage::DistanceAB(int itemA_index, int itemB_index){
        int i_x,i_y;
        if (itemA_index == itemB_index) {
            return 0;
        }
        if (itemA_index==99) {
            swap(itemA_index,itemB_index);
        }
        if (itemB_index==99) {
            i_x = mid_x;
            i_y = mid_y;
        }else{
            i_x = aStage->items().operator[](itemB_index).destination().x;
            i_y = aStage->items().operator[](itemB_index).destination().y;
        }
        return allMap[itemA_index][i_y][i_x];
    }
    
    int nStage::DistanceAxy(int a_index, int _x, int _y){
        return allMap[a_index][_y][_x];
    }
    
    bool nStage::overWeight(){
        for (int t = 0; t<4; ++t) {
            int w = BagWeight(bag[t]);
            if (w>15) {
                return true;
            }
        }
        return false;
    }
    
    int nStage::BagWeight(vector<int> b){
        int w = 0;
        REP(i, b.size()){
            w += ItemWeight(b[i]);
        }
        return w;
    }
    
    void nStage::StartBagChange(){
        int c[4];
        for (int i=0; i<4; ++i) {
            c[i] = int(bag[i].size());
        }
        int cnt = c[0]+c[1]+c[2]+c[3];
        int maxd_bagi = 0;
        int maxd_v = 0;
        int p[3];
        if (cnt==0) {
            for (int i=0; i<int(bag[4].size()); ++i) {
                int d =DistanceAxy(bag[4][i], mid_x, mid_y);
                if (maxd_v<d) {
                    maxd_bagi = i;
                    maxd_v = d;
                }
            }
            bag[0].push_back(bag[4][maxd_bagi]);
            bag[4].erase(bag[4].begin()+maxd_bagi);
            c[0]++;
            cnt++;
        }
        if (cnt==1) {
            for (int i=0; i<4; ++i) {
                if (bag[i].size()!=0) {
                    p[0] = bag[i][0];
                    break;
                }
            }
            for (int i=0; i<int(bag[4].size()); ++i) {
                maxd_v = 0;
                int d = DistanceAB(p[0], bag[4][i]);
                d += DistanceAB(99, bag[4][i]);
                if (maxd_v < d) {
                    maxd_bagi = i;
                    maxd_v = d;
                }
            }
            for (int i =0; i < 4; ++i) {
                if (c[i]==0) {
                    bag[i].push_back(bag[4][maxd_bagi]);
                    bag[4].erase(bag[4].begin()+maxd_bagi);
                    c[i]++;
                    cnt++;
                    break;
                }
            }
        }
        if (cnt==2) {
            int d = 0;
            maxd_v = 0;
            int q = 0;
            for (int i=0; i<4; ++i) {
                if (bag[i].size()==1) {
                    p[q] = bag[i][0];
                    q++;
                }
            }
            for (int i=0; i<int(bag[4].size()); ++i) {
                d += DistanceAB(p[0], bag[4][i]);
                d += DistanceAB(p[1], bag[4][i]);
                d += DistanceAB(99, bag[4][i]);
                if (maxd_v < d) {
                    maxd_bagi = i;
                    maxd_v = d;
                }
            }
            for (int i =0; i < 4; ++i) {
                if (c[i]==0) {
                    bag[i].push_back(bag[4][maxd_bagi]);
                    bag[4].erase(bag[4].begin()+maxd_bagi);
                    c[i]++;
                    cnt++;
                    break;
                }
            }
        }
        if (cnt==3) {
            int d = 0;
            maxd_v = 0;
            int q = 0;
            for (int i=0; i<4; ++i) {
                if (bag[i].size()==1) {
                    p[q] = bag[i][0];
                    q++;
                }
            }
            for (int i=0; i<int(bag[4].size()); ++i) {
                d += DistanceAB(p[0], bag[4][i]);
                d += DistanceAB(p[1], bag[4][i]);
                d += DistanceAB(p[2], bag[4][i]);
                d += DistanceAB(99, bag[4][i]);
                if (maxd_v < d) {
                    maxd_bagi = i;
                    maxd_v = d;
                }
            }
            for (int i =0; i < 4; ++i) {
                if (c[i]==0) {
                    bag[i].push_back(bag[4][maxd_bagi]);
                    bag[4].erase(bag[4].begin()+maxd_bagi);
                    c[i]++;
                    cnt++;
                    break;
                }
            }
            for (int i = 0; i<1; ++i) {
                for (int t=0; t<4; ++t) {
                    d = 0;
                    int min_d = 1000000000;
                    int min_i = 0;
                    for (int j = 0; j<int(bag[4].size()); ++j) {
                        d = DistanceAB(bag[t][i], bag[4][j]);
                        if (min_d > d) {
                            min_d = d;
                            min_i = j;
                        }
                    }
                    bag[t].push_back(bag[4][min_i]);
                    bag[4].erase(bag[4].begin()+min_i);
                }
            }
        }
        return;
    }
    
    //------------------------------------------------------------------------------
    /// 各ステージ開始時に呼び出されます。
    ///
    /// ここで、各ステージに対して初期処理を行うことができます。
    ///
    /// @param[in] aStage 現在のステージ。
    void Answer::Init(const Stage& aStage)
    {
        nStage t;
        t.getStage(aStage);
        t.solve();
//        t.ForceSolve();
        /////////////////////////////////////////////////////////////////
        //cout << "---------- stage no " << stage_n << "---------------" << endl;
        //                for (int i=0; i<5; i++) {
        //                    printf("%d  ",int(rBag[i].size()));
        //                }
    }
    
    
    //------------------------------------------------------------------------------
    /// 各配達時間帯開始時に呼び出されます。
    ///
    /// ここで、この時間帯に配達する荷物をトラックに積み込みます。
    /// どの荷物をトラックに積み込むかを決めて、引数の aItemGroup に対して
    /// setItem で荷物番号を指定して積み込んでください。
    ///
    /// @param[in] aStage 現在のステージ。
    /// @param[in] aItemGroup 荷物グループ。
    void Answer::InitPeriod(const Stage& aStage, ItemGroup& aItemGroup)
    {
        for (int i=0; i<int(rBag[aStage.period()].size()); ++i) {
            if (aStage.getTransportState(rBag[aStage.period()][i])==TransportState_NotTransported){
                aItemGroup.addItem(rBag[aStage.period()][i]);
                //                HPC_PRINT("詰め込み荷物: %d\n", rBag[aStage.period()][i]);
            }
        }
    }
    
    //------------------------------------------------------------------------------
    /// 各ターンでの動作を返します。
    ///
    /// @param[in] aStage 現在ステージの情報。
    ///
    /// @return これから行う動作を表す Action クラス。
    Action Answer::GetNextAction(const Stage& aStage)
    {
        Action ac = rAct[aStage.period()].front();
        rAct[aStage.period()].pop();
        //        HPC_PRINT("time: %d  Action: %u  %dx%d 残りの荷物: ",aStage.period(),ac,aStage.truck().pos().x,aStage.truck().pos().y);
        //        cout<<bitset<32>(aStage.truck().itemGroup().getBits())<<endl;
        return ac;
    }
    
    //------------------------------------------------------------------------------
    /// 各配達時間帯終了時に呼び出されます。
    ///
    /// ここで、この時間帯の終了処理を行うことができます。
    ///
    /// @param[in] aStage 現在のステージ。
    /// @param[in] aStageState 結果。Playingならこの時間帯の配達完了で、それ以外なら、何らかのエラーが発生した。
    /// @param[in] aCost この時間帯に消費した燃料。エラーなら0。
    void Answer::FinalizePeriod(const Stage& aStage, StageState aStageState, int aCost)
    {
        if (aStageState == StageState_Failed) {
            // 失敗したかどうかは、ここで検知できます。
            //printf("☓");
        }else{
            //printf("○");
        }
    }
    
    //------------------------------------------------------------------------------
    /// 各ステージ終了時に呼び出されます。
    ///
    /// ここで、各ステージに対して終了処理を行うことができます。
    ///
    /// @param[in] aStage 現在のステージ。
    /// @param[in] aStageState 結果。Completeなら配達完了で、それ以外なら、何らかのエラーが発生した。
    /// @param[in] aScore このステージで獲得したスコア。エラーなら0。
    void Answer::Finalize(const Stage& aStage, StageState aStageState, int aScore)
    {
                printf("%03d ",stage_n);
        if (aStageState == StageState_Failed) {
            failed_stage_n++;
                        printf("☓ 00000000 %08d\n",total_score);
        }
        else if (aStageState == StageState_TurnLimit) {
            // ターン数オーバーしたかどうかは、ここで検知できます。
        }else{
            successd_stage_n++;
            total_score += n_score;
                        printf("○ %08d %08d\n",n_score,total_score);
        }
        stage_n ++;
    }
}