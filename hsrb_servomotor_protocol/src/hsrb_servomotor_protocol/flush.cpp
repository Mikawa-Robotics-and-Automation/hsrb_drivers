/*
Copyright (c) 2017 TOYOTA MOTOR CORPORATION
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
#include <fstream>
#include <iostream>
#include <string>
#include <boost/lexical_cast.hpp>
#include <hsrb_servomotor_protocol/exxx_network.hpp>
#include <hsrb_servomotor_protocol/exxx_protocol.hpp>
#include <hsrb_servomotor_protocol/exxx_reprograming.hpp>

namespace {
const int64_t  kNetworkTimeout     = 10000000;
const int64_t  kNetworkTick        = 10000;
const uint32_t kExxxflushArgcMax   = 7;         /// exxx_flushコマンド 引数の数の最大値
const uint32_t kExxxflushArgcMin   = 4;         /// exxx_flushコマンド 引数の数の最小値
const uint32_t kFlushAxisIDNum     = 11;        /// リプログラミング軸番号の総数
const uint8_t  kFlushAxisInvalidID = 0xFF;      /// リプログラミング無効軸番号
const boost::array<uint8_t, kFlushAxisIDNum> FlushAxisIDTable = {{11, 12, 13, 21, 22, 23, 24, 25, 31, 32, 41}};
const boost::array<uint8_t, kFlushAxisIDNum> FlushBootAxisIDTable =
  {{0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x41, 0x42}};
const int64_t kReproBootWaitTime           = 500000000;     /// 通常リプロ プロップアンプの起動待ち時間
const int64_t kCompulsionReproBootWaitTime = 10000000000;    /// 強制リプロ プロップアンプの起動待ち時間
}  // anonymous namespace


int main(int argc, char** argv) {
  using hsrb_servomotor_protocol::INetwork;
  using hsrb_servomotor_protocol::ExxxNetwork;
  using hsrb_servomotor_protocol::ExxxProtocol;
  using hsrb_servomotor_protocol::ExxxReprograming;
  using hsrb_servomotor_protocol::ExxxWarningCategory;
  if ((argc < kExxxflushArgcMin) || (argc > kExxxflushArgcMax)) {
    std::cout << "USAGE: exxx_flush DEV ID FILENAME [OPTION]\n"
              << "\n"
              << "OPTION: \n"
              << "  -u, --usb usb485を利用．\n"
              << "  -c, --compulsion compulsion Reprogramming．\n"
              << "  -b115200, -b3000000 Baudrate．\n" << std::endl;
    return EXIT_SUCCESS;
  }

  boost::system::error_code error;
  try {
    std::string devicename(argv[1]);
    std::string filename(argv[3]);
    uint8_t id = boost::lexical_cast<int>(argv[2]);
    bool is_usb = false;
    bool is_compulsion = false;
    bool is_baudrate_specified = false;
    uint8_t axis_id = kFlushAxisInvalidID;
    uint32_t baudrate = B3000000;
    int32_t bootloader_version;
    int64_t boot_timeout;

    // 軸番号入力値をPROPアンプのブート部の軸番号へ変換
    for (uint32_t i = 0; i < kFlushAxisIDNum; ++i) {
      if (id == FlushAxisIDTable[i]) {
        axis_id = FlushBootAxisIDTable[i];
        break;
      }
    }
    if (axis_id == kFlushAxisInvalidID) {
      std::cerr << argv[2] << " invalid value" << std::endl;
      return EXIT_FAILURE;
    }

    std::string read_data;
    std::string flush_data;

    if ((filename.size() >= 5) && (filename.compare(filename.size() - 4, 4, ".mot") == 0)) {
      // ファイル読み込み
      std::ifstream ifs(filename.c_str());
      if (!ifs) {
        std::cerr << filename << " not found" << std::endl;
        return EXIT_FAILURE;
      }
      while (getline(ifs, read_data)) {
        flush_data = flush_data + read_data;
      }
      // 書き込みファイルのフォーマット確認。先頭2byteがSレコードフォーマット(S0～S9)であるかを確認する。
      if ((flush_data.size() < 2) || (flush_data[0] != 'S') || (flush_data[1] < '0') || (flush_data[1] > '9')) {
        std::cerr << filename << " format error" << std::endl;
        return EXIT_FAILURE;
      }
      bootloader_version = 1;
    } else if ((filename.size() >= 5) && (filename.compare(filename.size() - 4, 4, ".bin") == 0)) {
      std::ifstream ifs(filename.c_str(), std::ios::binary);
      if (!ifs) {
        std::cerr << filename << " not found" << std::endl;
        return EXIT_FAILURE;
      }
      std::ostringstream ostrm;
      ostrm << ifs.rdbuf();
      flush_data = ostrm.str();
      bootloader_version = 0;
    } else {
      std::cerr << "Unknown file type : " << filename << std::endl;
      return EXIT_FAILURE;
    }

    if (argc > kExxxflushArgcMin) {
      for (uint32_t i = kExxxflushArgcMin; i < argc; ++i) {
        std::string arg(argv[i]);
        if ((arg == "-u") || (arg == "--usb")) {
          is_usb = true;
        }
        if ((arg == "-c") || (arg == "--compulsion")) {
          is_compulsion = true;
        }
        // オプションの先頭2文字が-bであるか確認
        if (arg.compare(0, 2, "-b") == 0) {
          if ((arg != "-b115200") && (arg != "-b3000000")) {
            // 未対応のボーレートが指定された場合は、エラーを返す。
            std::cerr << arg << " invalid value" << std::endl;
            return EXIT_FAILURE;
          }
          // ボーレート未設定の場合は、引数で指定されたボーレートを設定する
          if (!is_baudrate_specified) {
            if (arg == "-b115200") {
              baudrate = B115200;
            } else {
              baudrate = B3000000;
            }
            is_baudrate_specified = true;
          }
        }
      }
    }

    if (is_compulsion) {
      std::cout << "Please release the stop button．\n" << std::flush;
      boot_timeout = kCompulsionReproBootWaitTime;
    } else {
      INetwork::Ptr network(new ExxxNetwork(devicename, error, is_usb, kNetworkTimeout, kNetworkTick));
      if (error) {
        std::cerr << error.message() << std::endl;
        return EXIT_FAILURE;
      }
      boost::shared_ptr<ExxxProtocol> protocol(new ExxxProtocol(network));

      error = protocol->Reset(id);
      if (error.value() != boost::system::errc::success) {
        std::cerr << error.message() << std::endl;
        return EXIT_FAILURE;
      }
      protocol.reset();
      boot_timeout = kReproBootWaitTime;
    }

    ExxxReprograming repro(devicename, is_usb, baudrate, kNetworkTimeout, kNetworkTick);
    error = repro.Open();
    if (error) {
      std::cerr << error.message() << std::endl;
      return EXIT_FAILURE;
    }

    std::cout << "Waiting boot "<< std::flush;
    error = repro.WaitBoot(bootloader_version, axis_id, boot_timeout);
    std::cout << std::endl;
    if (error.value() == boost::system::errc::no_message) {
      std::cerr << "Boot no response" << std::endl;
      return EXIT_FAILURE;
    } else if (error.value() == boost::system::errc::value_too_large) {
      std::cerr << "Boot wait time overflow" << std::endl;
      return EXIT_FAILURE;
    } else if (error.value() == boost::system::errc::not_supported) {
      std::cerr << "Boot loader does not supported this file format." << std::endl;
      return EXIT_FAILURE;
    } else if (error) {
      std::cerr << error.message() << std::endl;
      return EXIT_FAILURE;
    } else {
      // PROPアンプブート部起動正常終了
    }

    error = repro.Erase();
    if (error.value() == boost::system::errc::timed_out) {
      std::cerr << "Erase timeout" << std::endl;
      return EXIT_FAILURE;
    } else if (error.value() == boost::system::errc::no_message) {
      std::cerr << "Erase no response" << std::endl;
      return EXIT_FAILURE;
    } else if (error.value() == boost::system::errc::value_too_large) {
      std::cerr << "Erase wait time overflow" << std::endl;
      return EXIT_FAILURE;
    } else if (error) {
      std::cerr << error.message() << std::endl;
      return EXIT_FAILURE;
    } else {
      // イレース正常終了
    }

    std::cout << "Flush " << std::flush;
    error = repro.Flush(flush_data);
    std::cout << std::endl;
    if (error.value() == boost::system::errc::timed_out) {
      std::cerr << "Failed to Flush" << std::endl;
      return EXIT_FAILURE;
    } else if (error.value() == boost::system::errc::no_message) {
      std::cerr << "Flush no response" << std::endl;
      return EXIT_FAILURE;
    } else if (error.value() == boost::system::errc::value_too_large) {
      std::cerr << "Flush wait time overflow" << std::endl;
      return EXIT_FAILURE;
    } else if (error) {
      std::cerr << error.message() << std::endl;
      return EXIT_FAILURE;
    } else {
      // ファームウェア書き込み正常終了
    }

    error = repro.Run();
    if (error.value() == boost::system::errc::no_message) {
      std::cerr << "Run no response" << std::endl;
      return EXIT_FAILURE;
    } else if (error.value() == boost::system::errc::value_too_large) {
      std::cerr << "Run wait time overflow" << std::endl;
      return EXIT_FAILURE;
    } else if (error) {
      std::cerr << error.message() << std::endl;
      return EXIT_FAILURE;
    } else {
      // リプログラミング正常終了
      std::cout << "Success Flush" << std::endl;
      return EXIT_SUCCESS;
    }
  } catch (const boost::bad_lexical_cast& ex) {
    std::cerr << ex.what() << std::endl;
    return EXIT_FAILURE;
  } catch (...) {
    std::cerr << "Unexpected catch!" << std::endl;
    return EXIT_FAILURE;
  }
}
