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

using namespace std;

namespace hpc {
    int stage_n;                // ステージナンバー
    int successd_stage_n;
    int failed_stage_n;
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
    private:
        vector<vector<vector<int>>> allMap;  // allMap[基準点][y][x] = Pos(基準点)から(x,y)までの距離
        vector<vector<int>> bag;             // bag[時間帯] = 荷物リスト
        vector<queue<Action>> act;           // act[時間帯] = アクションリスト
        unordered_map <string, string>mp;
        int mid_x,mid_y;                                // 中心点
        void SetAllMap();                               // allMapを埋める
        void SetBag();                                  // 荷物リストからbag[時間帯]に入れる
        void SetAction();                               // 配送リストからアクションを埋める
        void SetActionStart2A(int time,int p);            // 中心点からPos(p)までのアクションを埋める > nStage.act
        void SetActionA2B(int time,int p, int n, int m);  // Pos(p)から(n,m)までのアクションを埋める > nStage.act
        void SetActionB2End(int time,int p);             // Pos(p)から(中心点までのアクションを埋める　> nStage.act
        void CopyGlovalANS();                             // rAct, rBag にコピーする
        void Greedy();                                    // bag[-1]の振り分けの全探索
        void ForceRoot(int t);                            // ルート決定　全探索 bag[t]を指定する
        void GreedyRoot();                                // ルート決定　貪欲法
        void GreedyRootT(int t);                          // ルート決定　貪欲法 bag(t)
        void Opt2(int t);                                 // ルート改善  2_Opt法(距離)
        void Opt2Fuel(int t);                             // ルート改善  2_Opt法(燃料)
        void ReverseRoot(int t);                          // ルート改善　逆順を試す(消費燃料を見る)
        int FuelCostR(vector<int> root);                  // ルートの消費燃料
        int DistanceAB(int a_index, int itemB_index);      // 点A,点Bとの距離
        int DistanceAxy(int a_index, int x, int y);        // 点A,点(x,y)との距離
        int BagWeight(vector<int> b);                      // バッグの重さ
        int ItemWeightBagIndex(int time,int bag_index);    // 重さ
        int ItemWeight(int item_index);                    // 重さ2
        bool overWeight();                                 // 荷物が15を超えているか
        long long Pow(int x, int y);                       // power
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
        if (bag[4].size()==0) {
            REP(t, 4){
                ForceRoot(t);
            }
        }else{
            Greedy();
        }
        SetAction();
        CopyGlovalANS();
    }

    
    void nStage::Greedy(){
        vector<vector<int>> init_bag(5);
        vector<vector<int>> max_bag(4);
        for (int i = 0; i<5; ++i) {
            copy(bag[i].begin(),bag[i].end(),back_inserter(init_bag[i]));
        }
        int min_fuel = inf;
        vector<int> delivery_time;
        ////////////////////////ランダム回数//// 提出時は 16385
        for (int q=0; q<6385; ++q) {
            int i = q;
            //　荷物(-1)がZ以上のときはランダム
            int Z = 3;
            if (bag[4].size()>Z) {
                i=rand()%Pow(4, int(bag[4].size()));
            }else{
                // 荷物(-1)が３以下の時、ループが全探索回数をすぎたら終了
                if (i>Pow(4, int(bag[4].size()))) {
                    break;
                }
            }
            //   bagを初期値に戻す
            for (int j = 0; j<4; ++j) {
                bag[j].clear();
                for (int k = 0; k<int(init_bag[j].size()); ++k) {
                    bag[j].push_back(init_bag[j][k]);
                }
            }
            // 数字に基づいて　配達時間を決定
            delivery_time.clear();
            for (int j=0; j<int(bag[4].size()); ++j) {
                delivery_time.push_back(i/Pow(4,j)%4);
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
            if (bag[4].size()>Z) {
                GreedyRoot();
            }else{
                REP(t, 4){ForceRoot(t);}
            }
            // 消費燃料の計算
            int total_fuel = 0;
            for (int t=0; t<4; ++t) {
                total_fuel += FuelCostR(bag[t]);
            }
            if (total_fuel<min_fuel) {
                min_fuel = total_fuel;
                for (int k=0; k<4; ++k) {
                    max_bag[k].clear();
                    for (int l=0; l<int(bag[k].size()); ++l) {
                        max_bag[k].push_back(bag[k][l]);
                    }
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
    
    
    
    
    //    rootから消費燃料
    int nStage::FuelCostR(vector<int> root){
        if (root.size()==0) {
            return 0;
        }
        int truck_weight = 3;
        for (int i=0; i<int(root.size()); ++i) {
            truck_weight += ItemWeight(root[i]);
        }
//        assert(truck_weight<=18);
        int fuel_consumption = 0;
        fuel_consumption += (truck_weight*DistanceAxy(root[0], mid_x, mid_y));
        truck_weight -= ItemWeight(root[0]);
        for (int i=0; i<int(root.size()-1); ++i) {
            fuel_consumption += (truck_weight*DistanceAB(root[i], root[i+1]));
            truck_weight -= ItemWeight(root[i+1]);
        }
//        assert(truck_weight==3);
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
                        if (FuelCostR(bag[t])>FuelCostR(root)) {
                            bag[t].clear();
                            copy(root.begin(),root.end(),back_inserter(bag[t]));
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
        string sorted_key= "";
        vector<int> tmp_bag;
        copy(bag[time].begin(), bag[time].end(), back_inserter(tmp_bag));
        sort(tmp_bag.begin(),tmp_bag.end());
        for (int j=0; j<int(bag[time].size()); ++j) {
            sorted_key.push_back('a'+tmp_bag[j]);
        }
        if(mp.count(sorted_key) != 0) {
            // ハッシュが設定されている場合の処理
            string abcd_best = mp[sorted_key];
            for (int j=0; j<int(bag[time].size()); ++j) {
                bag[time][j] = abcd_best.at(j)-'a';
            }
        } else {
            // keyが設定されてない時
            vector<int> min_bag;
            int min_fuel = inf;
            int s_fuel = 0;
            do{
                s_fuel = FuelCostR(tmp_bag);
                if (min_fuel>s_fuel){
                    min_fuel = s_fuel;
                    min_bag.clear();
                    copy(tmp_bag.begin(), tmp_bag.end(), back_inserter(min_bag));
                }
            }while (next_permutation(tmp_bag.begin(),tmp_bag.end()));
            string best_key;
            for (int j=0; j<int(bag[time].size()); ++j) {
                bag[time][j] = min_bag[j];
                best_key.push_back('a'+min_bag[j]);
            }
            mp[sorted_key] = best_key;
        }
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
        if (itemA_index==99 || itemB_index==99) {
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

    
    
    //------------------------------------------------------------------------------
    /// 各ステージ開始時に呼び出されます。
    ///
    /// ここで、各ステージに対して初期処理を行うことができます。
    ///
    /// @param[in] aStage 現在のステージ。
    void Answer::Init(const Stage& aStage)
    {
        stage_n ++;
        nStage t;
        t.getStage(aStage);
        t.solve();
        /////////////////////////////////////////////////////////////////
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
        if (aStageState == StageState_Failed) {
            failed_stage_n++;
        }
        else if (aStageState == StageState_TurnLimit) {
            // ターン数オーバーしたかどうかは、ここで検知できます。
        }else{
            successd_stage_n++;
        }
    }
}