#include "HPCAnswer.hpp"
#include "HPCMath.hpp"
#include "vector"
#include "queue"
#include "stack"
#include "iostream"
#include "bitset"
#include "algorithm"
#include "unordered_map"

using namespace std;

namespace hpc {
    
    int stag_i;                // ステージナンバー
    int successd_stage_n;
    int failed_stage_n;
    vector<queue<Action>> rAct;  // rAct[時間帯] = アクションリスト　nStage.act をコピーする
    vector<vector<int>> rBag;    // rBag[時間帯] = 荷物リスト        nStage.bag をコピーする
    const int x[]={-1,1,0,0};    // 上下左右のActionの選択用
    const int y[]={0,0,-1,+1};
    const Action a[]={Action_MoveLeft,Action_MoveRight,Action_MoveDown,Action_MoveUp};
    const Action reverse_a[]={Action_MoveRight,Action_MoveLeft,Action_MoveUp,Action_MoveDown};
    
    // 毎ステージで使うクラス　aStageの混合しないように注意
    class nStage{
        vector<vector<vector<int>>> allMap;  // allMap[基準点][y][x] = Pos(基準点)から(x,y)までの距離
        vector<vector<int>> bag;             // bag[時間帯] = 荷物リスト
        vector<queue<Action>> act;           // act[時間帯] = アクションリスト
        unordered_map <string, string>mp;
    public:
        nStage();                                       // 初期化? nStage::nStage()
        const Stage *aStage;                            // nStage内でaStageを参照するためのもの
        void getStage(const Stage& aStageSub);
        int mid_x,mid_y;                                // 中心点
        void calAllMap();                               // allMapを埋める
        void rootFromStart(int time,int p);             // 中心点からPos(p)までのアクションを埋める > nStage.act
        void rootAB(int time,int p, int n, int m);      // Pos(p)から(n,m)までのあkyションを埋める > nStage.act
        void rootToStart(int time,int p);               // Pos(p)から(中心点までのアクションを埋める　> nStage.act
        void putBag();                                  // 荷物リストからbag[時間帯]に入れる
        void ReOrderBagRoot();                          // bag[時間帯]の順番はそのままルートになる
                                                        // 0番目(重い荷物) N番目(近い荷物)
        void putAct();                                  // 配送リストからrootFromStart(), rootAB(), rootToStart()を呼び出す
        void ans();                                     // rAct, rBag にコピーする
        void Greedy();                                    // bag[-1]の振り分けの全探索
        // ルート探索 bag[n]の中身を入れ替える
        void RootSeach();                                 // ルート検索　親　TODO:後で作る
        void GreedyRoot();                                // ルート決定　貪欲法
        void Opt2(int t);                                 // ルート改善  2_Opt法
        void ReverseRoot(int t);                          // ルート改善　逆順を試す(消費燃料を見る)
        void ForceRoot(int t);                            // ルート決定　全探索 bag[t]を指定する
        void GreedyBag();                                 // bagの中の荷物を貪欲に並び替える
        string TimeOrderByList();
        // ↓使ってないもの(public)
    private:
        bool overWeight();                                 // 荷物が15を超えているか
        long long Pow(int x, int y);                       // power
        int RootCostFuel(vector<int> root);               // サークルの消費燃料
        int RootCostLength(vector<int> root);             // サークルの距離　TODO:あとで作る
        int ItemWeightBagIndex(int time,int bag_index);    // 重さ
        int ItemWeight(int item_index);                    // 重さ2
        int DistanceAB(int a_index, int itemB_index);      // 点A,点Bとの距離
        int DistanceAxy(int a_index, int x, int y);        // 点A,点(x,y)との距離
        // ↓使ってないもの(private)
    };
    
    nStage::nStage()
    :allMap(20,vector<vector<int>>(100,vector<int>(100))),bag(5),act(4)
    {}
    
    void nStage::getStage(const Stage& aStageSub){
        aStage=&aStageSub;
        mid_x = aStage->field().width()/2;
        mid_y = aStage->field().height()/2;
    }
    
    //     バッグに詰める　-1はbag[4]に
    void nStage::putBag(){
        for (int i = 0; i < aStage->items().count(); ++i) {
            //            HPC_PRINT("時間:%d 重さ:%d 配達先:%d,%d\n",aStage->items().operator[](i).period(),
            //            aStage->items().operator[](i).weight(),
            //            aStage->items().operator[](i).destination().x,
            //            aStage->items().operator[](i).destination().y);
            if (aStage->items().operator[](i).period()==-1) { // 時間未指定[-1]はbag[4]にいれる
                bag[4].push_back(i);
            }else{
                bag[aStage->items().operator[](i).period()].push_back(i);
            }
        }
    }
    
    
    void nStage::ReOrderBagRoot(){
        vector<int> temp_bag;
        for (int i=0; i<4; ++i) {
            // 重さソート
            for (int j=0; j<int(bag[i].size()); ++j) {
                for (int k=j+1; k<int(bag[i].size()); ++k) {
                    if (ItemWeightBagIndex(i, j)<ItemWeightBagIndex(i, k)) {
                        swap(bag[i][j], bag[i][k]);
                    }
                }
            }
            // 配達順決め
            // jと近いものを外側から内側にスワップしていく
            for (int j=0; j<int(bag[i].size()); ++j) {
                for (int k=j+2; k<int(bag[i].size());++k) {
                    if (DistanceAB(bag[i][j], bag[i][j+1])>DistanceAB(bag[i][j], bag[i][k])) {
                        swap(bag[i][j+1], bag[i][k]);
                    }else{
                        //                        ////printf("  ☓\n");
                    }
                }
            }
        }
    }
    
    //    (暫定)　かごの順番で回るためのActをつめる
    void nStage::putAct(){
        for (int i =0; i<4; ++i) {
            if (bag[i].size()>0) {
                rootFromStart(i, bag[i][0]);
            }
            if (int(bag[i].size())>1) {
                for (int j = 0; j<int(bag[i].size()-1); ++j) {
                    rootAB(i, bag[i][j+1], aStage->items().operator[](bag[i][j]).destination().x,
                           aStage->items().operator[](bag[i][j]).destination().y);
                }
            }
            if (bag[i].size()>0) {
                rootToStart(i, bag[i][bag[i].size()-1]);
            }
        }
    }
    
    //    (n,m)地点からpへの道順をact[time]につめる
    void nStage::rootAB(int time,int p, int n, int m){
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
                    act[time].push(a[i]);
                    break;
                }
            }
        }
    }
    
    void nStage::rootFromStart(int time,int p){
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
                    act[time].push(a[i]);
                    break;
                }
            }
        }
    }
    
    void nStage::rootToStart(int time,int p){
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
                    s.push(reverse_a[i]);
                    break;
                }
            }
        }
        while (s.empty()==false) {
            act[time].push(s.top());
            s.pop();
        }
    }
    
    void nStage::calAllMap(){
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
            //            for (int n = aStage->field().height()-1; n>=0; --n) {
            //                for (int m = 0; m<aStage->field().width(); ++m) {
            //                    if (aStage->field().isWall(m,n)==true) {
            //                        HPC_PRINT("..");
            //                    }else{;
            //                        HPC_PRINT("%02d",allMap[z][n][m]);
            //                    }
            //                }
            //                HPC_PRINT("\n");
            //            }
            //            HPC_PRINT("\n");
        }
        return;
    }
    
    void nStage::ans(){
        rAct = act;
        rBag = bag;
    }
    
    
    int nStage::ItemWeightBagIndex(int time,int bag_index){
        return ItemWeight(bag[time][bag_index]);
    }
    
    int nStage::ItemWeight(int item_index){
        return aStage->items().operator[](item_index).weight();
    }
    
    int nStage::DistanceAB(int itemA_index, int itemB_index){
        return allMap[itemA_index][aStage->items().operator[](itemB_index).destination().y][aStage->items().operator[](itemB_index).destination().x];
    }
    
    int nStage::DistanceAxy(int a_index, int _x, int _y){
        return allMap[a_index][_y][_x];
    }
    
    //    rootから消費燃料
    int nStage::RootCostFuel(vector<int> root){
        int truck_weight = 3;
        for (int i=0; i<int(root.size()); ++i) {
            truck_weight += ItemWeight(root[i]);
        }
        if (truck_weight>(15+3)) {
            ////printf("過積載");
            return 999999999;
        }
        int fuel_consumption = 0;
        fuel_consumption += truck_weight*DistanceAxy(root[0], mid_x, mid_y);
        truck_weight -= ItemWeight(root[0]);
        for (int i=0; i<int(root.size()-1); ++i) {
            fuel_consumption += truck_weight*DistanceAB(root[i], root[i+1]);
            truck_weight -= ItemWeight(root[i+1]);
        }
        truck_weight = 3;
        fuel_consumption += truck_weight*DistanceAxy(root[int(root.size()-1)], mid_x, mid_y);
        return fuel_consumption;
    }
    
    // ルート探索の貪欲法
    // 方針　中心点(mid_x,midy)から最も近いものを選んでいく
    void nStage::GreedyRoot(){
        for (int t=0; t<4; ++t) {
            // 荷物が1つの時は処理しない
            if (bag[t].size()<=1) {
                continue;
            }
            // 荷物が全探索できる個数なら全探索
            // 個々に処理
            if (bag[t].size()<=5) {
                ForceRoot(t);
                continue;
            }
            //
            int min_bag_index;
            for (int i=0; i<int(bag[t].size()); ++i) {
                // i==0のときは中心点に近いものを探す
                min_bag_index = i;
                int min_distance=1000000000;
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
            ReverseRoot(t);
        }
        return;
    }
    
    
    // ルート改善　2_Opt法
    // 参考　http://www.geocities.jp/m_hiroi/light/pyalgo64.html
    void nStage::Opt2(int t){
        while (true) {
            int count = 0;
            for (int i = 0; i<int(bag[t].size()-3); ++i) {
                int i1 = i + 1;
                for (int j=i+2; j<int(bag[t].size()-1); ++j) {
                    //                    if (j==int(allMap[t].size()-1) {
                    //                        // 0をふくんだベクターが必要
                    //                    }
                    int j1 = j + 1;
                    int l1 = 0;
                    int l2 = 0;
                    int l3 = 0;
                    int l4 = 0;
                    if ((i!=0) | (j1!=0)) {
                        l1 = DistanceAB(bag[t][i], bag[t][i1]);
                        l2 = DistanceAB(bag[t][j], bag[t][j1]);
                        l3 = DistanceAB(bag[t][i], bag[t][j]);
                        l4 = DistanceAB(bag[t][i1], bag[t][j1]);
                    }
                    if (l1+l2>l3+l4) {
                        swap(bag[t][i1], bag[t][j]);
                        count += 1;
                    }
                }
            }
            if (count == 0) {
                break;
            }
            
        }
        return;
    }
    
    // ルート改善　逆順のチェック
    void nStage::ReverseRoot(int t){
        vector<int> root;
        copy(bag[t].begin(),bag[t].end(),back_inserter(root));
        reverse(root.begin(),root.end());
        if (RootCostFuel(bag[t])>RootCostFuel(root)) {
            bag[t].clear();
            copy(root.begin(), root.end(), back_inserter(bag[t]));
        }
        return;
    }
    
    
    // ルート全探索
    void nStage::ForceRoot(int i){
            string sorted_key;
            vector<int> tmp_bag, min_bag;
            copy(bag[i].begin(), bag[i].end(), back_inserter(tmp_bag));
            sort(tmp_bag.begin(),tmp_bag.end());
            for (int j=0; j<int(bag[i].size()); ++j) {
                sorted_key.push_back('a'+tmp_bag[j]);
            }
            if(mp.count(sorted_key) != 0) {
                // ハッシュが設定されている場合の処理
                string abcd_best = mp[sorted_key];
                for (int j=0; j<int(bag[i].size()); ++j) {
                    bag[i][j] = abcd_best.at(j)-'a';
                }
            } else {
                // keyが設定されてない時
                int min_fuel = 1000000000;
                int s_fuel = 0;
                do{
                    s_fuel = RootCostFuel(tmp_bag);
                    if (min_fuel>s_fuel){
                        min_fuel = s_fuel;
                        min_bag.clear();
                        copy(tmp_bag.begin(), tmp_bag.end(), back_inserter(min_bag));
                    }
                }while (next_permutation(tmp_bag.begin(),tmp_bag.end()));
                string best_key;
                for (int j=0; j<int(bag[i].size()); ++j) {
                    bag[i][j] = min_bag[j];
                    best_key.push_back('a'+min_bag[j]);
                }
                mp[sorted_key] = best_key;
            }
    }
    
    void nStage::GreedyBag(){
        for (int i=0; i<4; ++i) {
            if(bag[i].size()<=1){continue;}
            string sorted_key;
            vector<int> tmp_bag, min_bag;
            copy(bag[i].begin(), bag[i].end(), back_inserter(tmp_bag));
            sort(tmp_bag.begin(),tmp_bag.end());
            for (int j=0; j<int(bag[i].size()); ++j) {
                sorted_key.push_back('a'+tmp_bag[j]);
            }
            if(mp.count(sorted_key) != 0) {
                // ハッシュが設定されている場合の処理
                string abcd_best = mp[sorted_key];
                for (int j=0; j<int(bag[i].size()); ++j) {
                    bag[i][j] = abcd_best.at(j)-'a';
                }
            } else {
                // keyが設定されてない時
                int min_fuel = 1000000000;
                int s_fuel = 0;
                do{
                    s_fuel = RootCostFuel(tmp_bag);
                    if (min_fuel>s_fuel){
                        min_fuel = s_fuel;
                        min_bag.clear();
                        copy(tmp_bag.begin(), tmp_bag.end(), back_inserter(min_bag));
                    }
                }while (next_permutation(tmp_bag.begin(),tmp_bag.end()));
                string best_key;
                for (int j=0; j<int(bag[i].size()); ++j) {
                    bag[i][j] = min_bag[j];
                    best_key.push_back('a'+min_bag[j]);
                }
                mp[sorted_key] = best_key;
            }
        }
    }
    
    long long nStage::Pow(int _x, int _y){
        long long r = 1;
        for (int i=0; i<_y; ++i) {
            r *= _x;
        }
        return r;
    }
    
    void nStage::Greedy(){
        ////printf("荷物: ");
        for (int i=0; i<4; ++i) {
            //printf("%02d  ",int(bag[i].size()));
        }
        //printf("(%d)\n",int(bag[4].size()));
        if (bag[4].size()==0) {
            GreedyBag();
            return;
        }
        vector<vector<int>> init_bag(5);
        vector<vector<int>> max_bag(4);
        for (int i = 0; i<5; ++i) {
            copy(bag[i].begin(),bag[i].end(),back_inserter(init_bag[i]));
        }
        
        int min_fuel = 100000000;
        vector<int> delivery_time;
        for (int q=0; q<6385; ++q) {////////////////////////ランダム回数//// 提出時は 16385
            int i = q;
            // 荷物(-1)が４つ以上の時はランダム
            if (bag[4].size()>3) {
                i=rand()%Pow(4, int(bag[4].size()));
            }else{
                if (i>Pow(4, int(bag[4].size()))) {
                    break;
                }
            }
            delivery_time.clear();
            for (int j=0; j<int(bag[4].size()); ++j) {
                delivery_time.push_back(i/Pow(4,j)%4);
            }
            //          ここに処理
            for (int j = 0; j<4; ++j) {
                bag[j].clear();
                for (int k = 0; k<int(init_bag[j].size()); ++k) {
                    bag[j].push_back(init_bag[j][k]);
                }
            }
            for (int j = 0; j<int(bag[4].size()); ++j) {
                bag[delivery_time[j]].push_back(bag[4][j]);
            }
            //            ここまででNEWバッグの生成完了
            if (overWeight()) {
                continue;
            }
//            GreedyBag(); // 配送順の最適化
            GreedyRoot();
            int total_fuel = 0;
            for (int j=0; j<4; ++j) {
                if (bag[j].size()==0) {
                    continue;
                }
                total_fuel += RootCostFuel(bag[j]);
            }
            //            int score = aStage->field().width() * aStage->field().height() * aStage->items().count() * 10000 / total_fuel;
            //            //printf("score = %d\n",score);
            if (total_fuel<min_fuel) {
                min_fuel = total_fuel;
                for (int k=0; k<4; ++k) {
                    max_bag[k].clear();
                    for (int l=0; l<int(bag[k].size()); ++l) {
                        max_bag[k].push_back(bag[k][l]);
                    }
                }
            }
            //            bagを初期値に戻す
            for (int j = 0; j<4; ++j) {
                bag[j].clear();
                for (int k = 0; k<int(init_bag[j].size()); ++k) {
                    bag[j].push_back(init_bag[j][k]);
                }
            }
        }
        for (int i=0; i<4; ++i) {
            bag[i].clear();
            for (int j=0; j<int(max_bag[i].size()); ++j) {
                bag[i].push_back(max_bag[i][j]);
            }
        }
        return;
    }
    

    
    bool nStage::overWeight(){
        int w ;
        for (int j = 0; j<4; ++j) {
            w = 0;
            for (int k=0; k<int(bag[j].size()); ++k) {
                w+=ItemWeight(bag[j][k]);
            }
            if (w>15) {
                return true;
            }
        }
        return false;
    }
    
    
    //    ========================================================================== solve
    void solve(const Stage& aStage){
        
        nStage t;
        t.getStage(aStage);
        //printf("----------------------------------↓ %d ↓----------------\n",stag_i++);
        t.putBag();
        //        t.RePutBag();
        t.calAllMap();
        //        t.ReOrderBagRoot();
        //        t.GreedyBag();
        t.Greedy();
        t.putAct();
        t.ans();
    }
    
    
    //------------------------------------------------------------------------------
    /// 各ステージ開始時に呼び出されます。
    ///
    /// ここで、各ステージに対して初期処理を行うことができます。
    ///
    /// @param[in] aStage 現在のステージ。
    void Answer::Init(const Stage& aStage)
    {
        solve(aStage);
        //        HPC_PRINT("debug++++++++++++++++++++++++++++++++++++++++++\n");
        //        for (int i=0; i<4; i++) {
        //            HPC_PRINT("時間帯: %d 荷物の数: %lu   (",i,rBag[i].size());
        //            for (int j = 0; j < int(rBag[i].size()); ++j) {
        //                HPC_PRINT("%d  ",rBag[i][j]);
        //            }
        //            HPC_PRINT(")\n");
        //        }
        //        HPC_PRINT("rAct[0].size()  %lu\n",rAct[0].size());
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
        //        if (aStage.period() == 0) {
        //            return;
        //        }
        //        for (int i = 0; i < aStage.items().count(); ++i) {
        //            // まだ配達されてない荷物かどうか調べる
        //            if (aStage.getTransportState(i) == TransportState_NotTransported) {
        //                // 配達されてない荷物なので積み込む
        //                aItemGroup.addItem(i);
        //            }
        //        }
        //        HPC_PRINT("つめ込み作業===============================\n");
        //        HPC_PRINT("time: %d\n",aStage.period());
        
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
        //        static Random random; // デフォルトのシード値を使う
        //        static Pos prev; // 初期値は重要ではない。(前のゲームの値が残っていても気にしない)
        //        for (int retry = 0; ; ++retry) {
        //            Action a = static_cast<Action>(random.randTerm(4));
        //            Pos nextPos = aStage.truck().pos().move(a);
        //            if (aStage.field().isWall(nextPos) == false) { // 動けるか
        //                if (retry < 50 && nextPos == prev) {
        //                    // 前にいた場所を避ける。
        //                    // これで、同じような場所をウロウロしてなかなか進まないのを防げる。
        //                    // ただし、50回やっても見つからないときは、諦める。
        //                    continue;
        //                }
        //                prev = aStage.truck().pos();
        //                return a;
        //            }
        //        }
        //
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
        if (aStageState == StageState_Failed) {
            // 失敗したかどうかは、ここで検知できます。
            //printf("\nno.%03d: 失敗",stag_i);
            failed_stage_n++;
        }
        else if (aStageState == StageState_TurnLimit) {
            // ターン数オーバーしたかどうかは、ここで検知できます。
        }else{
            //printf("\nno.%03d: 成功",stag_i);
            successd_stage_n++;
        }
        //printf(" (%07d)\nTOTAL: %d/%d\n",aStage.score(),successd_stage_n,successd_stage_n+failed_stage_n);
    }
}