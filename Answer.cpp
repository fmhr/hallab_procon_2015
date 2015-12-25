#include "HPCAnswer.hpp"
#include "HPCMath.hpp"
#include "vector"
#include "queue"
#include "stack"
#include "iostream"
#include "bitset"
#include <algorithm>

using namespace std;

namespace hpc {
    
    int stageNum;                // ステージナンバー
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
        void RePutBag();                                // bag[時間帯]の最適化(荷物[-1]を割り振る
        void ReOrderBagRoot();                          // bag[時間帯]の順番はそのままルートになる　0番目(重い荷物) N番目(近い荷物)
        void putAct();                                  // 配送リストからrootFromStart(), rootAB(), rootToStart()を呼び出す
        void ans();                                     // rAct, rBag にコピーする
        int ItemWeight(int time,int item_index);
        int DistanceAB(int itemA_index, int itemB_index);
    };
    
    nStage::nStage()
    :allMap(20,vector<vector<int>>(100,vector<int>(100))),bag(5),act(4)
    {}
    
    void nStage::getStage(const Stage& aStageSub){
        aStage=&aStageSub;
        mid_x = aStage->field().width()/2;
        mid_y = aStage->field().height()/2;
    }
    
    //    (暫定)　はいるだけ荷物を積める=================== nStage::putBag()
    void nStage::putBag(){        //        バッグに詰める　-1はbag[4]に
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
    
    void nStage::RePutBag(){
        vector<int> w(4);
        for (int i=0; i<4; ++i) {
            for (int j=0; j<int(bag[i].size()); ++j) {
                w[i] += aStage->items().operator[](bag[i][j]).weight();
            }
        }
        for (int i=0; i<int(bag[4].size()); ++i) {
            int min_bag = 0;
            int min_bag_w = 100;
            for (int j=0; j<4; ++j) {
                if (min_bag_w>w[j]){
                    min_bag_w = w[j];
                    min_bag = j;
                }
            }
            w[min_bag] += aStage->items().operator[](bag[4][i]).weight();
            bag[min_bag].push_back(bag[4][i]);
        }
    }
    
    void nStage::ReOrderBagRoot(){
        vector<int> temp_bag;
        for (int i=0; i<4; ++i) {
            // 重さソート
            for (int j=0; j<int(bag[i].size()); ++j) {
                for (int k=j+1; k<int(bag[i].size()); ++k) {
                    if (ItemWeight(i, j)<ItemWeight(i, k)) {
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
                        //                        printf("  ☓\n");
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
    
    int nStage::ItemWeight(int time,int item_index){
        return aStage->items().operator[](bag[time][item_index]).weight();
    }
    
    int nStage::DistanceAB(int itemA_index, int itemB_index){
        return allMap[itemA_index][aStage->items().operator[](itemB_index).destination().y][aStage->items().operator[](itemB_index).destination().x];
    }
    
    //    ==============================================================================================
    
    
    //    ========================================================================== solve
    void solve(const Stage& aStage){
        stageNum++;
        nStage t;
        t.getStage(aStage);
        //        HPC_PRINT("----------new stage %d------------------------------------------\n",stageNum);
        t.putBag();
        t.RePutBag();
        t.calAllMap();
        t.ReOrderBagRoot();
        t.putAct();
        t.ans();
        //        HPC_PRINT("中心:　%d x %d\n",t.mid_x,t.mid_y);
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
        }
        else if (aStageState == StageState_TurnLimit) {
            // ターン数オーバーしたかどうかは、ここで検知できます。
        }
        
        //        printf("スコア  %d\n",aStage.score());
    }
}

