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
#ifndef HSRB_SERVOMOTOR_PROTOCOL_CONTROL_TABLE_ITEM_DESCRIPTOR_HPP_
#define HSRB_SERVOMOTOR_PROTOCOL_CONTROL_TABLE_ITEM_DESCRIPTOR_HPP_

#include <stdint.h>
#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>

namespace hsrb_servomotor_protocol {

class ControlTableItemDescriptor {
 public:
  typedef boost::shared_ptr<ControlTableItemDescriptor> Ptr;
  typedef boost::shared_ptr<const ControlTableItemDescriptor> ConstPtr;

  /// 型のタイプ
  enum ValueType {
    kUInt8 = 1,
    kInt8,
    kUInt16,
    kInt16,
    kUInt32,
    kInt32,
    kUInt64,
    kInt64,
    kFloat,
    kDouble,
  };

  /// コンストラクタ
  /// @param[in] type このエントリの型
  /// @param[in] initial_address このエントリの先頭アドレス
  /// @param[in] attribute このエントリの属性
  /// @param[in] coefficient_mks MKS単位系への変換係数
  ControlTableItemDescriptor(ValueType type, uint16_t initial_address, const std::string& attribute,
                             double coefficient_mks);

  /// byte列をMKS単位のdoubleに変換
  /// @param[in] bytes 変換する値
  /// @param[out] success 成功，失敗 引数に与えたbytesが短いとエラーを返す
  /// @return 成否 bytesのサイズがtype()のバイト数と等しく無いと失敗
  bool ConvertToMKS(const std::vector<uint8_t>& bytes, double& mks_value_out) const;

  /// byte列をMKS単位のdoubleに変換
  /// @param[in] bytes_begin 変換する値へのポインタ
  /// @param[out] success 成功，失敗 引数に与えたbytesが短いとエラーを返す
  /// @return 成否 bytesのサイズがtype()のバイト数と等しく無いと失敗
  bool ConvertToMKS(const uint8_t* bytes_begin, double& mks_value_out) const;

  /// MKS単位をバイト列に変換
  /// @param[in] bytes 変換する値の先頭アドレス
  /// @param[out] MKS単位での返り値
  /// @return 成否 mks_valueがオーバーフローすると失敗
  bool ConvertToBytes(double mks_value, std::vector<uint8_t>& bytes_out) const;

  /// 型
  ValueType type() const { return type_; }

  /// 先頭アドレス
  uint16_t initial_address() const { return initial_address_; }

  /// このエントリの最終アドレス+1 (初期アドレス+バイト数)
  uint16_t final_address() const { return final_address_; }

  /// 属性
  std::string attribute() const { return attribute_; }

  /// バイト数
  uint8_t num_bytes() const { return num_bytes_; }

 private:
  const ValueType type_;

  const uint16_t initial_address_;

  uint16_t final_address_;

  const std::string attribute_;

  const double coefficient_mks_;

  double coefficient_table_;

  uint8_t num_bytes_;
};

}  // namespace hsrb_servomotor_protocol
#endif  // HSRB_SERVOMOTOR_PROTOCOL_CONTROL_TABLE_ITEM_DESCRIPTOR_HPP_
