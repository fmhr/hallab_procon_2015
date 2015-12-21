//------------------------------------------------------------------------------
/// @file
/// @brief Random クラス
/// @author ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2015 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------
#pragma once

#include "HPCTypes.hpp"

namespace hpc {

    //------------------------------------------------------------------------------
    /// 乱数生成の機能を提供します。
    ///
    /// 乱数列は、シードの値によって一意に定められます。
    class Random
    {
    public:
        Random();
        Random(uint aSeedX, uint aSeedY, uint aSeedZ, uint aSeedW);

        int randTerm(int aTerm);                ///< [0, aTerm) の範囲で乱数を取得します。
        int randMinTerm(int aMin, int aTerm);   ///< [aMin, aTerm) の範囲で乱数を取得します。
        int randMinMax(int aMin, int aMax);     ///< [aMin, aMax] の範囲で乱数を取得します。

    private:
        uint mSeedX;            ///< 乱数のシード
        uint mSeedY;            ///< 乱数のシード
        uint mSeedZ;            ///< 乱数のシード
        uint mSeedW;            ///< 乱数のシード

        uint randCoreU32();     ///< 内部で乱数を発生させます。
    };
}
//------------------------------------------------------------------------------
// EOF
