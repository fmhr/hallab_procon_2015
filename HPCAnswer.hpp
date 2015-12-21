//------------------------------------------------------------------------------
/// @file
/// @brief    Answer クラス
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2015 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------
#pragma once

#include "HPCAction.hpp"
#include "HPCStage.hpp"

namespace hpc {

    //------------------------------------------------------------------------------
    /// ゲームの解答を表します。
    ///
    /// 参加者は Answer.cpp にこのクラスのメンバ関数を実装することで、
    /// プログラムを作成します。
    class Answer
    {
    public:

        static void Init(const Stage& aStage);                                              ///< 各ステージ開始時に呼び出されます。
        static void InitPeriod(const Stage& aStage, ItemGroup& aItemGroup);                 ///< 各配達時間帯開始時に呼び出されます。
        static Action GetNextAction(const Stage& aStage);                                   ///< 次の動作を決定します。
        static void FinalizePeriod(const Stage& aStage, StageState aStageState, int aCost); ///< 各配達時間帯終了時に呼び出されます。
        static void Finalize(const Stage& aStage, StageState aStageState, int aScore);      ///< 各ステージ終了時に呼び出されます。

    private:
        Answer();
    };
}
//------------------------------------------------------------------------------
// EOF
