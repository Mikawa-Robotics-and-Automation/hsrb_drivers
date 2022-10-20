/*
Copyright (c) 2016 TOYOTA MOTOR CORPORATION
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific
  prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/
#ifndef HSRB_SERVOMOTOR_PROTOCOL_CONTROL_TABLE_HPP_
#define HSRB_SERVOMOTOR_PROTOCOL_CONTROL_TABLE_HPP_

#include <string>
#include <vector>
#include <boost/cstdint.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/utility.hpp>

namespace hsrb_servomotor_protocol {

class ControlTableItemDescriptor;

class ControlTable : private boost::noncopyable {
 public:
  /// Loadの返り値
  enum ErrorCode {
    kSuccess = 0,      /// 成功
    kFileOpenError,    /// ファイルのオープンに失敗
    kColumnSizeError,  /// コントロールテーブルの要素数エラー
    kAlreadyRecorded,  /// 同じエントリが２つ以上存在する
    kBadType,          /// 間違った型指定が行われた
  };

  ControlTable();

  ~ControlTable();

  /// 定義ファイルを渡してmd5sumを計算する
  /// @return ファイルの読み込みの成否 kSuccessで成功
  ErrorCode CalculateMd5Sum(const std::string& definition_file);

  /// 定義ファイルを渡して初期化する
  /// @return ファイルの読み込みの成否 kSuccessで成功
  ErrorCode Load(const std::string& definition_file);

  /// このコントロールテーブルのmd5sumを取得
  std::vector<uint8_t> GetMd5Sum();

  /// コントロールテーブルのエントリ名からプロパティを取得
  /// 存在しない名前を与えたらshared_ptrの空を返す
  boost::shared_ptr<ControlTableItemDescriptor> ReferItemDescriptor(const std::string& entry) const;

 private:
  /// 実装
  class ControlTableImpl;

  /// pimpl
  boost::scoped_ptr<ControlTableImpl> pimpl_;
};

}  // namespace hsrb_servomotor_protocol
#endif  // HSRB_SERVOMOTOR_PROTOCOL_CONTROL_TABLE_HPP_
