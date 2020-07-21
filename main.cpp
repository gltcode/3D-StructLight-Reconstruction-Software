/*
Copyright (c) 2012, Daniel Moreno and Gabriel Taubin
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Brown University nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL DANIEL MORENO AND GABRIEL TAUBIN BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "Application.hpp"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

//#include <iostream>
//#include <string>
//#include <fstream> 
//#include <iostream>
//using namespace std;

int main(int argc, char *argv[])
{
    //ofstream txt01;   //保存的文件
    //ofstream txt02;   //保存的文件
    //string temp;
    //int index = 0; //用于判断奇偶
    //std::string BasePath = "D:\\Toolkit\\PCL\\PCL 1.9.1\\lib";

    //ifstream fin(BasePath+"\\0.txt");

    //while (!fin.eof()) // 若未到文件结束一直循环
    //{

    //    std::cout << index << std::endl;
    //    //std::string lineString;
    //    std::getline(fin, temp);
    //    //getline(txtfile, temp); //一行一行读取
    //    std::cout << temp << std::endl;
    //    if (index % 2 == 0)     //判断除以2的余数，即为奇偶的判断
    //    {
    //        txt01.open(BasePath+"\\1.txt", ios::app);
    //        txt01 << temp;
    //        txt01 << endl;
    //        txt01.close();
    //    }
    //    else {
    //        txt02.open(BasePath+"\\2.txt", ios::app);
    //        txt02 << temp;
    //        txt02 << endl;
    //        txt02.close();
    //    }
    //    index++;
    //}
    //fin.close(); //关闭文件
    //txt01.close();
    //txt02.close();

    //return 0;


    Application app(argc, argv);
    return app.exec();
}
