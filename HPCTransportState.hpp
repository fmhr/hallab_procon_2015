//------------------------------------------------------------------------------
/// @file
/// @brief    TransportState 列挙型
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2015 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------
#pragma once

namespace hpc {

    //------------------------------------------------------------------------------
    /// @brief 配達状況を表します。
    enum TransportState {
        TransportState_NotTransported, ///< 未配達
        TransportState_Transporting,   ///< 今の時間帯に配達
        TransportState_Transported,    ///< 前の時間帯までに配達済み

        TransportState_TERM
    };
}
//------------------------------------------------------------------------------
// EOF
