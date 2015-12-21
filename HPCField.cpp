//------------------------------------------------------------------------------
/// @file
/// @brief    HPCField.hpp の実装
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2015 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください

//------------------------------------------------------------------------------

#include "HPCField.hpp"

namespace hpc {
    //------------------------------------------------------------------------------
    /// クラスのインスタンスを生成します。
    Field::Field()
        : mWidth(0)
        , mHeight(0)
        , mWalls()
    {
    }

    //------------------------------------------------------------------------------
    /// 初期設定を行います。
    void Field::setup(int aWidth, int aHeight, int aDensity, Random& aRandom)
    {
        HPC_RANGE_ASSERT_MIN_MAX_I(aWidth, Parameter::FieldWidthMin, Parameter::FieldWidthMax);
        HPC_RANGE_ASSERT_MIN_MAX_I(aHeight, Parameter::FieldHeightMin, Parameter::FieldHeightMax);
        HPC_ASSERT(aWidth % 4 == 3);
        HPC_ASSERT(aHeight % 4 == 3);
        mWidth = aWidth;
        mHeight = aHeight;
        for (int i = 0; i < mHeight; ++i) {
            for (int j = 0; j < mWidth; ++j) {
                mWalls[i][j] = Parameter::PeriodAllMask;
            }
        }
        // まずは営業所を通路にする。
        Pos office = officePos();
        mWalls[office.y][office.x] = 0;

        // すでに通路になっているところから、通路になるべきなのにまだなってないところに向かって掘る
        // ここで、あるマスから別のマスへの行き方が1通りに決まるようなマップになる
        int gx = (mWidth - 1) / 2;
        int gy = (mHeight - 1) / 2;
        int count = gx * gy - 1;
        while (count > 0) {
            int x = aRandom.randTerm(gx) * 2 + 1;
            int y = aRandom.randTerm(gy) * 2 + 1;
            if (mWalls[y][x] == 0) {
                Pos p(x, y);
                Action dir = static_cast<Action>(aRandom.randTerm(4));
                Pos center = p.move(dir);
                Pos next = center.move(dir);
                if (next.x >= 0 && next.x < aWidth && next.y >= 0 && next.y < aHeight) {
                    if (mWalls[next.y][next.x] != 0) {
                        mWalls[next.y][next.x] = 0;
                        mWalls[center.y][center.x] = 0;
                        count--;
                    }
                }
            }
        }

        // 壁密度に合わせて適当に掘る
        for (int i = 1; i < mHeight - 1; ++i) {
            for (int j = 1; j < mWidth - 1; ++j) {
                if ((i + j) % 2 == 1) {
                    if (aRandom.randTerm(100) >= aDensity) {
                        mWalls[i][j] = 0;
                    }
                }
            }
        }
    }

    //------------------------------------------------------------------------------
    /// フィールド情報を設定します。
    void Field::set(const Field& aField)
    {
        mWidth = aField.mWidth;
        mHeight = aField.mHeight;
        for (int i = 0; i < Parameter::FieldHeightMax; ++i) {
            for (int j = 0; j < Parameter::FieldWidthMax; ++j) {
                mWalls[i][j] = aField.mWalls[i][j];
            }
        }
    }

    //------------------------------------------------------------------------------
    /// @return 幅。
    int Field::width()const
    {
        return mWidth;
    }

    //------------------------------------------------------------------------------
    /// @return 高さ。
    int Field::height()const
    {
        return mHeight;
    }

    //------------------------------------------------------------------------------
    /// @param[in] aX    X座標。
    /// @param[in] aY    Y座標。
    ///
    /// @return 壁かどうか。
    bool Field::isWall(const int aX, const int aY) const
    {
        HPC_RANGE_ASSERT_MIN_UB_I(aX, 0, mWidth);
        HPC_RANGE_ASSERT_MIN_UB_I(aY, 0, mHeight);
        return mWalls[aY][aX] != 0;
    }

    //------------------------------------------------------------------------------
    /// @param[in] aPos    座標。
    ///
    /// @return 壁かどうか。
    bool Field::isWall(const Pos& aPos) const
    {
        return isWall(aPos.x, aPos.y);
    }

    //------------------------------------------------------------------------------
    /// @return 営業所の座標。
    Pos Field::officePos()const
    {
        return Pos((mWidth - 1) / 2, (mHeight - 1) / 2);
    }
}
//------------------------------------------------------------------------------
// EOF
