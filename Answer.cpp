//------------------------------------------------------------------------------
/// @file
/// @brief    HPCAnswer.hpp の実装 (解答記述用ファイル)
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2015 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------

#include "HPCAnswer.hpp"
#include "HPCMath.hpp"
#include "vector"
#include "queue"
#include "stack"
#include "iostream"
#include "bitset"

using namespace std;

/// プロコン問題環境を表します。
namespace hpc {
    
    int stageNum;
    vector<queue<Action>> rAct;
    vector<vector<int>> rBag;
//    ===============================================================================================
    
    const int x[]={-1,1,0,0};
    const int y[]={0,0,-1,+1};
    const Action a[]={Action_MoveLeft,Action_MoveRight,Action_MoveDown,Action_MoveUp};
    const Action reverse_a[]={Action_MoveRight,Action_MoveLeft,Action_MoveUp,Action_MoveDown};
    
    class nStage{
    public:
        vector<vector<int>> bag;
        vector<vector<vector<int>>> allMap;
        std::vector<std::queue<Action>> act;
        nStage();
        const Stage *aStage;
        void getStage(const Stage& aStageSub);
        int mid_x,mid_y;
        void calAllMap();
        void rootFromStart(int time,int p);
        void rootAB(int time,int p, int n, int m);
        void rootToStart(int time,int p);
        void putBag();
        void putAct();
        void ans();
    };
    
    nStage::nStage()
    : bag(5), act(4),
    allMap(20,vector<vector<int>>(100,vector<int>(100)))
    {}
    
    void nStage::getStage(const Stage& aStageSub){
        aStage=&aStageSub;
        mid_x = aStage->field().width()/2;
        mid_y = aStage->field().height()/2;
    }
    
    //    (暫定)　はいるだけ荷物を積める=================== nStage::putBag()
    void nStage::putBag(){
//        バッグに詰める　-1はbag[4]に
        for (int i = 0; i < aStage->items().count(); ++i) {
            HPC_PRINT("時間:%d 重さ:%d 配達先:%d,%d\n",aStage->items().operator[](i).period(), aStage->items().operator[](i).weight(), aStage->items().operator[](i).destination().x, aStage->items().operator[](i).destination().y);
            //            時間毎にバッグに詰める
            if (aStage->items().operator[](i).period()==-1) {
                bag[4].push_back(i);
            }else{
                bag[aStage->items().operator[](i).period()].push_back(i);
            }
        }
//          bag[4]の荷物を振り分ける
        std::vector<int> w (4);
        //        初期の0~3時間のをつめこむ
//        各時間の重量を数える
        for (int i=0; i<4; ++i) {
            for (int j=0; j<bag[i].size(); ++j) {
                w[i] += aStage->items().operator[](bag[i][j]).weight();
            }
        }
//            bag[4]の荷物をうつす
        for (int j = 0; j<bag[4].size(); ++j) {
            for (int k=0; k<4; ++k) {
                if (w[k]+aStage->items().operator[](bag[4][j]).weight()<15) {
                    w[k] += aStage->items().operator[](bag[4][j]).weight();
                    bag[k].push_back(bag[4][j]);
                    break;
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
            if (bag[i].size()>1) {
                for (int j = 0; j<bag[i].size()-1; ++j) {
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
            for (int n = aStage->field().height()-1; n>=0; --n) {
                for (int m = 0; m<aStage->field().width(); ++m) {
                    if (aStage->field().isWall(m,n)==true) {
                        HPC_PRINT("..");
                    }else{;
                        HPC_PRINT("%02d",allMap[z][n][m]);
                    }
                }
                HPC_PRINT("\n");
            }
            HPC_PRINT("\n");
        }
        return;
    }

    void nStage::ans(){
        rAct = act;
        rBag = bag;
    }
    
//    ==============================================================================================
//    std::vector<std::vector <int>> map (100,std::vector<int>(100));
//    std::vector<std::vector<int>> bag(5,std::vector<int>());
//    std::vector<std::queue<Action>> act(4);
//    vector<vector<queue<Action>>> rootAcx(20,vector<queue<Action>>(20,queue<Action>()));
//    int now_time=0;
//
//    
//    void calAllMap(const Stage& aStage,vector<vector<vector<int>>> &allMap){
//        const int x[] = {1,0,-1,0};
//        const int y[] = {0,1,0,-1};
//        queue<Pos> q;
//        for (int z=0; z<aStage.items().count(); ++z) {
//            allMap[z][aStage.items().operator[](z).destination().y][aStage.items().operator[](z).destination().x]=-1;
//            q.push(Pos(aStage.items().operator[](z).destination().x,
//                       aStage.items().operator[](z).destination().y));
//            while (q.empty()==false) {
//                for (int i=0; i<4; ++i) {
//                    int n_x = q.front().x+x[i];
//                    int n_y = q.front().y+y[i];
//                    if (aStage.field().isWall(n_x, n_y)==false){
//                        if (allMap[z][n_y][n_x]==0) {
//                            q.push(Pos(n_x,n_y));
//                            allMap[z][n_y][n_x] = allMap[z][q.front().y][q.front().x]+1;
//                        }
//                    }
//                }
////                for (int n = 0; n<aStage.field().height(); ++n) {
////                    for (int m = 0; m<aStage.field().width(); ++m) {
////                        if (aStage.field().isWall(m,n)==true) {
////                            HPC_PRINT("..");
////                        }else{;
////                            HPC_PRINT("%02d",allMap[z][n][m]);
////                        }
////                    }
////                    HPC_PRINT("\n");
////                }
////                HPC_PRINT("\n");
//                q.pop();
//            }
//        }
//        return;
//    }
//    
//
////    (x,y)地点からpへの道順をact[time]につめる
//    void rootToAB(int time,int p, int n, int m,vector<vector<vector<int>>> &allMap){
//        if (bag[time].size()==0) {
//            return;
//        }
//        int x[]={1,0,-1,0};
//        int y[]={0,1,0,-1};
//        Action a[]={Action_MoveRight,Action_MoveUp,Action_MoveLeft,Action_MoveRight};
//        int n_x = n;
//        int n_y = m;
//        int step = allMap[p][m][n];
//        while (step>0) {
//            for (int i=0; i<4; ++i) {
//                if (allMap[p][n_y+y[i]][n_x+x[i]]==step-1) {
//                    step -= 1;
//                    n_x = n_x+x[i];
//                    n_y = n_y+y[i];
//                    act[time].push(a[i]);
//                    HPC_PRINT("%d  %d\n",time,i);
//                    break;
//                }
//            }
//        }
//    }
//    
//    void rootFromStart(int time,int p,vector<vector<vector<int>>> &allMap,const Stage& aStage){
//        if (bag[time].size()==0) {
//            return;
//        }
//        int x[]={1,0,-1,0};
//        int y[]={0,1,0,-1};
//        Action a[]={Action_MoveLeft,Action_MoveRight,Action_MoveDown,Action_MoveUp};
//        int n_x = aStage.field().width()/2;
//        int n_y = aStage.field().height()/2;
//        int step = allMap[p][n_y][n_x];
//        while (step>0) {
//            for (int i=0; i<4; ++i) {
//                if (allMap[p][n_y+y[i]][n_x+x[i]]==step-1) {
//                    step -= 1;
//                    n_x = n_x+x[i];
//                    n_y = n_y+y[i];
//                    act[time].push(a[i]);
//                    HPC_PRINT("%d  %d\n",time,i);
//                    break;
//                }
//            }
//        }
//    }
//    
//    void rootToStart(int time,int p,vector<vector<vector<int>>> &allMap,const Stage& aStage){
//        if (bag[time].size()==0) {
//            return;
//        }
//        int x[]={1,0,-1,0};
//        int y[]={0,1,0,-1};
//        Action a[]={Action_MoveRight,Action_MoveUp,Action_MoveLeft,Action_MoveRight};
//        int n_x = aStage.field().width()/2;
//        int n_y = aStage.field().height()/2;
//        int step = allMap[p][n_y][n_x];
//        stack<Action> s;
//        while (step>0) {
//            for (int i=0; i<4; ++i) {
//                if (allMap[p][n_y+y[i]][n_x+x[i]]==step-1) {
//                    step -= 1;
//                    n_x = n_x+x[i];
//                    n_y = n_y+y[i];
//                    s.push(a[i]);
//                    break;
//                }
//            }
//        }
//        while (s.empty()==false) {
//            HPC_PRINT("%d  %d\n",time,s.top());
//            act[time].push(s.top());
//            s.pop();
//        }
//    }
//    
////    (暫定)　かごの順番で回るためのActをつめる
//    void putAct(const Stage& aStage,vector<vector<vector<int>>> &allMap){
//        for (int i =0; i<4; ++i) {
//            rootFromStart(i, 0, allMap, aStage);
//            if (bag[i].size()>1) {
//                for (int j = 0; j<bag[i].size()-1; ++j) {
//                    rootToAB(i, j+1, aStage.items().operator[](bag[i][j]).destination().x,
//                             aStage.items().operator[](bag[i][j]).destination().y, allMap);
//                }
//            }
//            rootToStart(i, int(bag[i].size()-1), allMap, aStage);
//        }
//        
//    }
//    
//    
//    
////    (暫定)　はいるだけ荷物を積める
//    void putBug(const Stage& aStage){
//        std::vector<int> w (4);
////        初期の0~3時間のをつめこむ
//        for (int i=0; i<4; ++i) {
//            for (int j=0; j<bag[i].size(); ++j) {
//                w[i] += aStage.items().operator[](bag[i][j]).weight();
//            }
//        }
//        for (int j = 0; j<bag[4].size(); ++j) {
//            for (int k=0; k<4; ++k) {
//                if (w[k]+aStage.items().operator[](bag[4][j]).weight()<15) {
//                    w[k] += aStage.items().operator[](bag[4][j]).weight();
//                    break;
//                }
//            }
//        }
//    }
//    
//    
//    
////  start -> end 距離
//    int bfs(const Stage& aStage, Pos start, Pos end){
//        HPC_PRINT("debug---bfs-----------------------\n");
//        int x[] = {1,0,-1,0};
//        int y[] = {0,1,0,-1};
//        std::vector<std::vector<int>> smap(100,std::vector<int>(100));
//        std::queue<Pos> q;
//        q.push(start);
//        while (q.empty()==false) {
//            if (q.front() == end) {
//                return smap[q.front().y][q.front().x];
//            }
//            for (int i=0; i<4; ++i) {
//                int n_x = q.front().x+x[i];
//                int n_y = q.front().y+y[i];
//                if (aStage.field().isWall(n_x, n_y)==false && smap[n_y][n_x]==0) {
//                    q.push(Pos(n_x,n_y));
//                    smap[n_y][n_x] = smap[q.front().y][q.front().x]+1;
//                }
//            }
//            q.pop();
//        }
////        逆から辿ってルートを確定(複数あるときは最初のものを選ぶ)
//        return 0;
//    }
    
//    ========================================================================== solve
    void solve(const Stage& aStage){
        stageNum++;
        nStage t;
        t.getStage(aStage);
        HPC_PRINT("----------new stage %d------------------------------------------\n",stageNum);
        t.putBag();
        t.calAllMap();
        t.putAct();
        t.ans();
        
        HPC_PRINT("中心:　%d x %d\n",t.mid_x,t.mid_y);
//        HPC_PRINT("debug++++++++++++++++++++++++++++++++++++++++++\n");
//        for (int i=0; i<4; i++) {
//            HPC_PRINT("時間帯: %d 荷物の数: %lu\n",i,t.bag[i].size());
//        }
        
//        
//        HPC_PRINT("----------new stage------------------------------------------\n");
//        HPC_PRINT("%d x %d\n",aStage.field().width(), aStage.field().height());
//        HPC_PRINT("荷物数: %d\n",aStage.items().count());
//        //    各配達先からマップ全体のステップ数
//        vector<vector<vector<int>>> allMap (20,vector<vector<int>>(100,vector<int>(100)));
//        calAllMap(aStage,allMap);
//        Pos start, end;
////        時間毎のバッグ
//        start.x = aStage.field().width()/2;
//        start.y = aStage.field().height()/2;
//        for (int i = 0; i < aStage.items().count(); ++i) {
//            HPC_PRINT("時間:%d 重さ:%d 配達先:%d,%d\n",aStage.items().operator[](i).period(), aStage.items().operator[](i).weight(), aStage.items().operator[](i).destination().x, aStage.items().operator[](i).destination().y);
////            時間毎にバッグに詰める
//            if (aStage.items().operator[](i).period()==-1) {
//                bag[4].push_back(i);
//            }else{
//                bag[aStage.items().operator[](i).period()].push_back(i);
//            }
//            putBug(aStage);
//            putAct(aStage, allMap);
////            スタート地点からの距離を出す
//            end.x = aStage.items().operator[](i).destination().x;
//            end.y = aStage.items().operator[](i).destination().y;
////            HPC_PRINT("スタート地点との距離   %d\n", bfs(aStage,start,end));
//            HPC_PRINT("スタート地点との距離2　%d\n", allMap[i][start.y][start.x]);
//        }
//
//        HPC_PRINT("----------bugの中身をチェック----------\n");
//        for (int i=0; i<5; ++i) {
//            HPC_PRINT("time:%d 荷物の数:%lu   ",i,bag[i].size());
//        }
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
        HPC_PRINT("debug++++++++++++++++++++++++++++++++++++++++++\n");
        for (int i=0; i<4; i++) {
            HPC_PRINT("時間帯: %d 荷物の数: %lu   (",i,rBag[i].size());
            for (int j = 0; j < rBag[i].size(); ++j) {
                HPC_PRINT("%d  ",rBag[i][j]);
            }
            HPC_PRINT(")\n");
        }
        HPC_PRINT("rAct[0].size()  %lu\n",rAct[0].size());
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
        HPC_PRINT("つめ込み作業===============================\n");
        HPC_PRINT("time: %d\n",aStage.period());
        
        for (int i=0; i<rBag[aStage.period()].size(); ++i) {
            if (aStage.getTransportState(rBag[aStage.period()][i])==TransportState_NotTransported){
                aItemGroup.addItem(rBag[aStage.period()][i]);
                HPC_PRINT("詰め込み荷物: %d\n", rBag[aStage.period()][i]);
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
        Action a = rAct[aStage.period()].front();
        rAct[aStage.period()].pop();
        HPC_PRINT("time: %d  Action: %u  %dx%d 残りの荷物: ",aStage.period(),a,aStage.truck().pos().x,aStage.truck().pos().y);
        cout<<bitset<32>(aStage.truck().itemGroup().getBits())<<endl;
        return a;
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
            HPC_PRINT("ステージ失敗☆");
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
            HPC_PRINT("失敗☆");
        }
        else if (aStageState == StageState_TurnLimit) {
            // ターン数オーバーしたかどうかは、ここで検知できます。
        }
        
        std::printf("スコア  %d\n",aStage.score());
    }
}

//------------------------------------------------------------------------------
// EOF
