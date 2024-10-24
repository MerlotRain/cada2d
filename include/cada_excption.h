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

#ifndef CADA_EXCEPTION_H
#define CADA_EXCEPTION_H

#include <stdexcept>
#include <string>

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4251) // warning C4251: needs to have dll-interface to
                                // be used by clients of class
#pragma warning(disable : 4275) // warning C4275: non-DLL-interface
#endif                          // std::exception used as base for DLL-inter

namespace cada {

class Exception : public std::runtime_error {
public:
    Exception() : std::runtime_error("unknown error") {}

    Exception(std::string const &msg) : std::runtime_error(msg) {}

    Exception(std::string const &name, std::string const &msg)
        : std::runtime_error(name + ": " + msg)
    {
    }
};

} // namespace cada

#endif