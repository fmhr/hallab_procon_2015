//------------------------------------------------------------------------------
/// @file
/// @brief    Field クラス
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2015 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------
#pragma once

//------------------------------------------------------------------------------
#include "HPCCommon.hpp"
#include "HPCParameter.hpp"
#include "HPCPos.hpp"
#include "HPCRandom.hpp"

namespace hpc {

    //------------------------------------------------------------------------------
    /// 矩形のフィールドを表します。
    class Field
    {
    public:
        Field();

        /// フィールドを生成する。
        /// @param[in]      aWidth   フィールドの幅
        /// @param[in]      aHeight  フィールドの高さ
        /// @param[in]      aDensity 壁密度。0なら消せる壁は全部消え、100なら、あるマスから別のマスへの行き方は1通りになる。
        /// @param[in,out]  aRandom  乱数
        void setup(int aWidth, int aHeight, int aDensity, Random& aRandom);

        void set(const Field& aField); ///< フィールド情報を設定します。

        int width()const;  ///< 幅。 0 <= Pos.x < width() 。
        int height()const; ///< 高さ。 0 <= Pos.y < height() 。

        bool isWall(int aX , int aY) const; ///< 指定位置が壁かどうかを取得。
        bool isWall(const Pos& aPos) const; ///< 指定位置が壁かどうかを取得。

        Pos officePos() const;        ///< 営業所の位置取得。

    private:
        int mWidth;
        int mHeight;
        int mWalls[Parameter::FieldHeightMax][Parameter::FieldWidthMax];
    };
}
//------------------------------------------------------------------------------
// EOF
