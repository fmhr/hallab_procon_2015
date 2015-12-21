//------------------------------------------------------------------------------
/// @file
/// @brief    TurnResult クラス
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2015 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------
#pragma once

#include "HPCParameter.hpp"
#include "HPCStageState.hpp"
#include "HPCPos.hpp"
#include "HPCItemGroup.hpp"

namespace hpc {

    //------------------------------------------------------------------------------
    /// 1ターンの実行に対する結果を表します。
    struct TurnResult 
    {
        TurnResult();

        void reset();                                                   ///< 情報を初期化します。
        void set(const TurnResult& aResult);                            ///< 値を上書き設定します。

        bool initPeriod;     ///< 積み込みターンか？
        Action action;       ///< アクション(積み込みターンではない場合のみ)
        Pos truckPos;        ///< トラックの位置
        ItemGroup itemGroup; ///< トラックに積まれている荷物
        int periodCost;      ///< この時間帯に消費した燃料
        int totalCost;       ///< この時間帯までに消費した燃料
        
        StageState state;   ///< 現在の状態
    };
}
//------------------------------------------------------------------------------
// EOF
