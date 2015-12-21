//------------------------------------------------------------------------------
/// @file
/// @brief HPCRandom.hpp の実装
/// @author ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2015 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------
#include "HPCRandom.hpp"

#include "HPCCommon.hpp"

namespace {
    /// @name シードの初期値
    ///
    /// @note シードの初期値は途中評価用です。
    ///       最終評価では異なるシードによる評価が行われます。
    ///       このシードの各要素の値を変えることで、プログラムが
    ///       いろいろなシードの値に耐えうるかを調べることができます。
    //@{
    const unsigned int DefaultSeedX = 0x31f9f4c6;
    const unsigned int DefaultSeedY = 0xc7871429;
    const unsigned int DefaultSeedZ = 0x492f765a;
    const unsigned int DefaultSeedW = 0xa3a3992f;
    //@}
}

namespace hpc {
    //------------------------------------------------------------------------------
    /// クラスのインスタンスを生成します。
    ///
    Random::Random()
        : mSeedX(DefaultSeedX)
        , mSeedY(DefaultSeedY)
        , mSeedZ(DefaultSeedZ)
        , mSeedW(DefaultSeedW)
    {
    }

    //------------------------------------------------------------------------------
    /// クラスのインスタンスを生成します。
    ///
    /// @param[in] aSeedX 乱数のシード。
    /// @param[in] aSeedY 乱数のシード。
    /// @param[in] aSeedZ 乱数のシード。
    /// @param[in] aSeedW 乱数のシード。
    Random::Random(uint aSeedX, uint aSeedY, uint aSeedZ, uint aSeedW)
        : mSeedX(aSeedX)
        , mSeedY(aSeedY)
        , mSeedZ(aSeedZ)
        , mSeedW(aSeedW)
    {
    }

    //------------------------------------------------------------------------------
    /// ランダムな整数値を [0, aTerm) の範囲で発生させます。
    ///
    /// @param[in] aTerm 乱数を発生させる範囲の上界。
    ///
    /// @return 発生させた一つの乱数。
    int Random::randTerm(int aTerm)
    {
        HPC_LB_ASSERT_I(aTerm, 0);
        return int(randCoreU32() & 0x7FFFFFFF) % aTerm;
    }

    //------------------------------------------------------------------------------
    /// ランダムな整数値を [aMin, aTerm) の範囲で発生させます。
    ///
    /// @param[in] aMin  乱数を発生させる範囲の最小値。
    /// @param[in] aTerm 乱数を発生させる範囲の上界。
    ///
    /// @return 発生させた一つの乱数。
    int Random::randMinTerm(int aMin, int aTerm)
    {
        HPC_MAX_ASSERT_I(aMin, aTerm - 1);
        return aMin + randTerm(aTerm - aMin);
    }

    //------------------------------------------------------------------------------
    /// ランダムな整数値を [aMin, aMax] の範囲で発生させます。
    ///
    /// @param[in] aMin  乱数を発生させる範囲の最小値。
    /// @param[in] aMax  乱数を発生させる範囲の最大値。
    ///
    /// @return 発生させた一つの乱数。
    int Random::randMinMax(int aMin, int aMax)
    {
        HPC_MAX_ASSERT_I(aMin, aMax);
        return aMin + randTerm(1 + aMax - aMin);
    }

    //------------------------------------------------------------------------------
    /// [0, UINT_MAX] の範囲をもつ乱数を内部で計算して乱数列を1つ進め、
    /// 現在の値を返します。
    ///
    /// @note 内部で状態が書き換わります。
    ///
    /// @return 現在の乱数列の値。
    uint Random::randCoreU32()
    {
        const uint t = (mSeedX ^ (mSeedX << 11));
        mSeedX = mSeedY;
        mSeedY = mSeedZ;
        mSeedZ = mSeedW;
        mSeedW = (mSeedW ^ (mSeedW >> 19)) ^ (t ^ (t>> 8));
        return mSeedW;
    }

    //------------------------------------------------------------------------------
}
//------------------------------------------------------------------------------
// EOF
