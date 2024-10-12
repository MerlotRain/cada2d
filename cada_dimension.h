/**
 * Copyright (c) 2024-present Merlot.Rain
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

#ifndef CADA_DIMENSION_H
#define CADA_DIMENSION_H

#include "cada_shape.h"

#include <string>

namespace cada {


class TextData
{
    std::string mText;
    Vec2d mPosition;
    Vec2d mAlignmentPoint;
    double mTextHeight;
    double mTextWidth;
    NS::VAlign mVerticalAlignment;
    NS::HAlign mHorizontalAlignment;
    NS::TextDrawingDirection mDrawingDirection;
    NS::TextLineSpacingStyle mLineSpacingStyle;
    double mLineSpacingFactor;
    std::string mFontFamily;
    std::string mFontFile;
    double mAngle;
    double mXScale;
    int mTextFlags;
};

struct DimensionStyle
{
    double mOverallScale;
    double mLinearMeasurementFactor;
    double mTextHeight;
    double mDimensionLineGap;
    double mArrowSize;
    double mExtensionLineExtension;
    double mExtensionLineOffset;

    int mTextPositionVertical;
    bool mTextHorizontal;
    double mDimensionLineIncrement;
    RGB mTextColor;

    double mTickSize;
    int mLinearFormat;
    int mDecimalPlaces;
    char mDecimalSeparator;
    int mZeroSuppression;
    int mAngularFormat;

    int mAngularDecimalPlaces;
    int mAngularZeroSuppression;
    std::string mArrowBlock;
};

class Dimension
{
    mutable Vec2d mDefinitionPoint;
    mutable Vec2d mTextPositionCenter;
    mutable Vec2d textPositionSide;

    NS::VAlign mValign;
    NS::HAlign mHalign;

    NS::TextLineSpacingStyle mLineSpacingStyle;
    double mLineSpacingFactor;

    std::string mText;
    std::string mUpperTolerance;
    std::string mLowerTolerance;
    std::string mFontFamily;
    mutable std::string mDimBlockName;

    mutable double mDefaultAngle;
    double mTextRotation;

    // style attribute

    bool mArrow1Fipped;
    bool mArrow2Fipped;

    bool mExtLineFix;
    double mExtLineFixLength;

    mutable bool mDirty;
    mutable TextData mTextData;
    mutable BBox mBoundingBox;

    mutable double mDimLineLength;
    mutable Vec2d mArrow1Pos;
    mutable Vec2d mArrow2Pos;

    mutable bool mAutoTextPos;
    mutable std::vector<Shape*> mShapes;
};

class DimAngular : public Dimension
{
};

class DimAngular2L : public DimAngular
{
    /** Start point of first extension line. */
    Vec2d mExtensionLine1Start;
    /** End point of first extension line. */
    Vec2d mExtensionLine1End;
    /** Start point of second extension line. End is definition point. */
    Vec2d mExtensionLine2Start;
    /** Arc position */
    Vec2d mDimArcPosition;
};

class DimAngular3P : public DimAngular
{
    Vec2d mCenter;
    /** End point of first extension line. */
    Vec2d mExtensionLine1End;
    /** End point of second extension line. */
    Vec2d mExtensionLine2End;
};

class DimArcLength : public DimAngular
{
    /** Start point of first extension line. */
    Vec2d mCenter;
    /** End point of first extension line. */
    Vec2d mExtensionLine1End;
    /** End point of second extension line. */
    Vec2d mExtensionLine2End;
    /** Arc position is definitionPoint */

    /** Arc symbol type 0: before, 1: above, 2: off */
    int mArcSymbolType;
};

class DimDiametric : public Dimension
{
    Vec2d mChordPoint;
};


class DimLinear : public Dimension
{
    Vec2d mExtensionPoint1;
    Vec2d mExtensionPoint2;
};

class DimAligned : public DimLinear
{

};

class DimRotated : public DimLinear
{

};

class DimOrdinate : public Dimension
{
    Vec2d mDefiningPoint;
    Vec2d mLeaderEndPoint;
    bool mXType;
};

class DimRadial : public Dimension
{
    Vec2d mChordPoint;
};


} // namespace cada

#endif