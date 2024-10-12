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

#include "cada_shape.h"

using namespace cada;


/******* inlines *****/
 Mat::TransformationType Mat::inline_type() const
{
    if (m_dirty == TxNone)
        return static_cast<TransformationType>(m_type);
    return type();
}

 bool Mat::isAffine() const
{
    return inline_type() < TxProject;
}
 bool Mat::isIdentity() const
{
    return inline_type() == TxNone;
}

 bool Mat::isInvertible() const
{
    return !qFuzzyIsNull(determinant());
}

 bool Mat::isScaling() const
{
    return type() >= TxScale;
}
 bool Mat::isRotating() const
{
    return inline_type() >= TxRotate;
}

 bool Mat::isTranslating() const
{
    return inline_type() >= TxTranslate;
}

 qreal Mat::determinant() const
{
    return m_matrix[0][0] * (m_matrix[2][2] * m_matrix[1][1] - m_matrix[2][1] * m_matrix[1][2]) -
           m_matrix[1][0] * (m_matrix[2][2] * m_matrix[0][1] - m_matrix[2][1] * m_matrix[0][2]) +
           m_matrix[2][0] * (m_matrix[1][2] * m_matrix[0][1] - m_matrix[1][1] * m_matrix[0][2]);
}
 qreal Mat::m11() const
{
    return m_matrix[0][0];
}
 qreal Mat::m12() const
{
    return m_matrix[0][1];
}
 qreal Mat::m13() const
{
    return m_matrix[0][2];
}
 qreal Mat::m21() const
{
    return m_matrix[1][0];
}
 qreal Mat::m22() const
{
    return m_matrix[1][1];
}
 qreal Mat::m23() const
{
    return m_matrix[1][2];
}
 qreal Mat::m31() const
{
    return m_matrix[2][0];
}
 qreal Mat::m32() const
{
    return m_matrix[2][1];
}
 qreal Mat::m33() const
{
    return m_matrix[2][2];
}
 qreal Mat::dx() const
{
    return m_matrix[2][0];
}
 qreal Mat::dy() const
{
    return m_matrix[2][1];
}

QT_WARNING_PUSH
QT_WARNING_DISABLE_FLOAT_COMPARE

 Mat &Mat::operator*=(qreal num)
{
    if (num == 1.)
        return *this;
    m_matrix[0][0] *= num;
    m_matrix[0][1] *= num;
    m_matrix[0][2] *= num;
    m_matrix[1][0] *= num;
    m_matrix[1][1] *= num;
    m_matrix[1][2] *= num;
    m_matrix[2][0] *= num;
    m_matrix[2][1] *= num;
    m_matrix[2][2] *= num;
    if (m_dirty < TxScale)
        m_dirty = TxScale;
    return *this;
}
 Mat &Mat::operator/=(qreal div)
{
    if (div == 0)
        return *this;
    div = 1/div;
    return operator*=(div);
}
 Mat &Mat::operator+=(qreal num)
{
    if (num == 0)
        return *this;
    m_matrix[0][0] += num;
    m_matrix[0][1] += num;
    m_matrix[0][2] += num;
    m_matrix[1][0] += num;
    m_matrix[1][1] += num;
    m_matrix[1][2] += num;
    m_matrix[2][0] += num;
    m_matrix[2][1] += num;
    m_matrix[2][2] += num;
    m_dirty     = TxProject;
    return *this;
}
 Mat &Mat::operator-=(qreal num)
{
    if (num == 0)
        return *this;
    m_matrix[0][0] -= num;
    m_matrix[0][1] -= num;
    m_matrix[0][2] -= num;
    m_matrix[1][0] -= num;
    m_matrix[1][1] -= num;
    m_matrix[1][2] -= num;
    m_matrix[2][0] -= num;
    m_matrix[2][1] -= num;
    m_matrix[2][2] -= num;
    m_dirty     = TxProject;
    return *this;
}

QT_WARNING_POP

 bool qFuzzyCompare(const Mat& t1, const Mat& t2) noexcept
{
    return qFuzzyCompare(t1.m11(), t2.m11())
        && qFuzzyCompare(t1.m12(), t2.m12())
        && qFuzzyCompare(t1.m13(), t2.m13())
        && qFuzzyCompare(t1.m21(), t2.m21())
        && qFuzzyCompare(t1.m22(), t2.m22())
        && qFuzzyCompare(t1.m23(), t2.m23())
        && qFuzzyCompare(t1.m31(), t2.m31())
        && qFuzzyCompare(t1.m32(), t2.m32())
        && qFuzzyCompare(t1.m33(), t2.m33());
}


/****** stream functions *******************/
#ifndef QT_NO_DATASTREAM
Q_GUI_EXPORT QDataStream &operator<<(QDataStream &, const Mat &);
Q_GUI_EXPORT QDataStream &operator>>(QDataStream &, Mat &);
#endif

#ifndef QT_NO_DEBUG_STREAM
Q_GUI_EXPORT QDebug operator<<(QDebug, const Mat &);
#endif
/****** end stream functions *******************/

// mathematical semantics
 QPoint operator*(const QPoint &p, const Mat &m)
{ return m.map(p); }
 QPointF operator*(const QPointF &p, const Mat &m)
{ return m.map(p); }
 QLineF operator*(const QLineF &l, const Mat &m)
{ return m.map(l); }
 QLine operator*(const QLine &l, const Mat &m)
{ return m.map(l); }
 QPolygon operator *(const QPolygon &a, const Mat &m)
{ return m.map(a); }
 QPolygonF operator *(const QPolygonF &a, const Mat &m)
{ return m.map(a); }
 QRegion operator *(const QRegion &r, const Mat &m)
{ return m.map(r); }

 Mat operator *(const Mat &a, qreal n)
{ Mat t(a); t *= n; return t; }
 Mat operator /(const Mat &a, qreal n)
{ Mat t(a); t /= n; return t; }
 Mat operator +(const Mat &a, qreal n)
{ Mat t(a); t += n; return t; }
 Mat operator -(const Mat &a, qreal n)
{ Mat t(a); t -= n; return t; }