//------------------------------------------------------------------------------
/// @file
/// @brief    HPCTurnResult.hpp の実装
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2015 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------

#include "HPCTurnResult.hpp"

#include "HPCCommon.hpp"

namespace hpc {

    //------------------------------------------------------------------------------
    /// クラスのインスタンスを生成します。
    TurnResult::TurnResult()
    {
        reset();
    }

    //------------------------------------------------------------------------------
    /// 値を既定の値で初期化します。
    void TurnResult::reset()
    {
        state = StageState_TERM;
    }

    //------------------------------------------------------------------------------
    /// 別の TurnResult で値を上書きします。
    /// 基本情報がすべて上書きされます。
    ///
    /// @param[in] aResult  上書きする TurnResult
    void TurnResult::set(const TurnResult& aResult)
    {
        initPeriod = aResult.initPeriod;
        action = aResult.action;
        truckPos = aResult.truckPos;
        itemGroup.set(aResult.itemGroup);
        periodCost = aResult.periodCost;
        totalCost = aResult.totalCost;
        state = aResult.state;
    }
}

//------------------------------------------------------------------------------
// EOF
