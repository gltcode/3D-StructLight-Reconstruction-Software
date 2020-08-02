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


#include "ProjectorWidget.hpp"
#include<opencv2\opencv.hpp>

#include <QApplication>
#include <QDesktopWidget>
#include <QPainter>

#include <stdio.h>
#include <iostream>
#include <assert.h>

#include "structured_light.hpp"
#include "Application.hpp"

ProjectorWidget::ProjectorWidget(QWidget* parent, Qt::WindowFlags flags) :
    QWidget(parent, flags),
    _screen(0),
    _current_pattern(-1),
    _pattern_count(4),
    _vbits(1),
    _hbits(1),
    _updated(false),
    _SpaceCodeImage()
{
}

ProjectorWidget::~ProjectorWidget()
{
    stop();
}

void ProjectorWidget::reset(void)
{
    _current_pattern = -1;
    _updated = false;
    _pixmap = QPixmap();
    _SpaceCodeImage = cv::Mat();
    emit new_image(_pixmap);
}

void ProjectorWidget::start(void)
{
    stop();
    reset();

    //validate screen
    QDesktopWidget * desktop = QApplication::desktop();
    int screens =  desktop->screenCount();
    if (_screen<0 || _screen>=screens)
    {   //error, fix it
        _screen = screens;
    }

    //display
    QRect screen_resolution = desktop->screenGeometry(_screen);
    move(QPoint(screen_resolution.x(), screen_resolution.y()));
    showFullScreen();

    //update bit count for the current resolution
    update_pattern_bit_count();
}

void ProjectorWidget::stop(void)
{
    hide();
    reset();
}

void ProjectorWidget::prev(void)
{
    if (_updated)
    {   //pattern not processed: wait
        return;
    }

    if (_current_pattern<1)
    {
        return;
    }

    _current_pattern--;
    _pixmap = QPixmap();
    update();
    QApplication::processEvents();
}

void ProjectorWidget::next(void)
{
    if (_updated)
    {   //pattern not processed: wait
        return;
    }

    if (finished())
    {
        return;
    }

    _current_pattern++;
    _pixmap = QPixmap();
    update();
    QApplication::processEvents();
}

bool ProjectorWidget::finished(void)
{
    if (APP->config.value("reconstruction/Code_Mode", Gray_Code) == Gray_Code)
    {
        return (_current_pattern + 2 > 2 + 4 * _pattern_count);
    }
    else if (APP->config.value("reconstruction/Code_Mode", Space_Code) == Space_Code)
    {
        return (_current_pattern > 2 * _pattern_count-2);
    }
}

void ProjectorWidget::paintEvent(QPaintEvent *)
{
    QPainter painter(this);
    //if (_pixmap.isNull())
    if (_current_pattern<0)
    {   //stopped
        QRectF rect = QRectF(QPointF(0,0), QPointF(width(),height()));
        painter.drawText(rect, Qt::AlignCenter, "No image");
    }
    else
    {
        bool updated = false;
        if (_pixmap.isNull())
        {   //update
            updated = true;
            make_pattern();
        }

        //draw
        QRectF rect = QRectF(QPointF(0,0), QPointF(width(),height()));
        painter.drawPixmap(rect, _pixmap, rect);

        if (updated)
        {   //notfy update
            _updated = true;
            emit new_image(_pixmap);
        }
    }
}

void ProjectorWidget::update_pattern_bit_count(void)
{
    if (APP->config.value("reconstruction/Code_Mode", Gray_Code) == Gray_Code)
    {
        int cols = width();
        int rows = height();

        //search bit number
        _vbits = 1;
        _hbits = 1;
        for (int i = (1 << _vbits); i < cols; i = (1 << _vbits)) { _vbits++; }
        for (int i = (1 << _hbits); i < rows; i = (1 << _hbits)) { _hbits++; }
        _pattern_count = std::min(std::min(_vbits, _hbits), _pattern_count);
        std::cerr << "[ GRAY CODE MODE]" << std::endl;
        std::cerr << " vbits " << _vbits << " / cols=" << cols << ", mvalue=" << ((1 << _vbits) - 1) << std::endl;
        std::cerr << " hbits " << _hbits << " / rows=" << rows << ", mvalue=" << ((1 << _hbits) - 1) << std::endl;
        std::cerr << " pattern_count=" << _pattern_count << std::endl;
    }
    else if (APP->config.value("reconstruction/Code_Mode", Space_Code) == Space_Code)
    {
        _pattern_count = 1;
        std::cerr << "[ SPACE CODE MODE]" << std::endl;
        std::cerr << " pattern_count=" << _pattern_count << std::endl;
    }
}

void ProjectorWidget::make_pattern(void)
{
    int cols = width();
    int rows = height();
    if (APP->config.value("reconstruction/Code_Mode", Gray_Code) == Gray_Code)
    {
    /*
    if (_current_pattern<1)
    {   //search bit number
        _vbits = 1;
        _hbits = 1;
        for (int i=(1<<_vbits); i<cols; i=(1<<_vbits)) { _vbits++; }
        for (int i=(1<<_hbits); i<rows; i=(1<<_hbits)) { _hbits++; }
        _pattern_count = std::min(std::min(_vbits, _hbits), _pattern_count);
        std::cerr << " vbits " << _vbits << " / cols="<<cols<<", mvalue="<< ((1<<_vbits)-1) << std::endl;
        std::cerr << " hbits " << _hbits << " / rows="<<rows<<", mvalue="<< ((1<<_hbits)-1) << std::endl;
        std::cerr << " pattern_count="<< _pattern_count << std::endl; 
    }
    */

    int vmask = 0, voffset = ((1<<_vbits)-cols)/2, hmask = 0, hoffset = ((1<<_hbits)-rows)/2, inverted = (_current_pattern%2)==0;

    // patterns
    // -----------
    // 00 white
    // 01 black
    // -----------
    // 02 vertical, bit N-0, normal
    // 03 vertical, bit N-0, inverted
    // 04 vertical, bit N-1, normal
    // 04 vertical, bit N-2, inverted
    // ..
    // XX =  (2*_pattern_count + 2) - 2 vertical, bit N, normal
    // XX =  (2*_pattern_count + 2) - 1 vertical, bit N, inverted
    // -----------
    // 2+N+00 = 2*(_pattern_count + 2) horizontal, bit N-0, normal
    // 2+N+01 horizontal, bit N-0, inverted
    // ..
    // YY =  (4*_pattern_count + 2) - 2 horizontal, bit N, normal
    // YY =  (4*_pattern_count + 2) - 1 horizontal, bit N, inverted

    if (_current_pattern<2)
    {   //white or black
        _pixmap = make_pattern(rows, cols, vmask, voffset, hmask, hoffset, inverted);
    }
    else if (_current_pattern<2*_pattern_count+2)
    {   //vertical
        int bit = _vbits - _current_pattern/2;
        vmask = 1<<bit;
        //std::cerr << "v# cp: " << _current_pattern << " bit:" << bit << " mask:" << vmask << std::endl;
        _pixmap = make_pattern(rows, cols, vmask, voffset, hmask, hoffset, !inverted);
    }
    else if (_current_pattern<4*_pattern_count+2)
    {   //horizontal
        int bit = _hbits + _pattern_count - _current_pattern/2;
        hmask = 1<<bit;
        //std::cerr << "h# cp: " << _current_pattern << " bit:" << bit << " mask:" << hmask << std::endl;
        _pixmap = make_pattern(rows, cols, vmask, voffset, hmask, hoffset, !inverted);
    }
    else
    {   //error
        assert(false);
        stop();
        return;
    }

    //_pixmap.save(QString("pat_%1.png").arg(_current_pattern, 2, 10, QLatin1Char('0')));
    }
    else if (APP->config.value("reconstruction/Code_Mode", Space_Code) == Space_Code)
    {
        int inverted = (_current_pattern % 2) == 1;

        _pixmap = make_space_pattern( rows,  cols, inverted);
    }
}

QPixmap ProjectorWidget::make_pattern(int rows, int cols, int vmask, int voffset, int hmask, int hoffset, int inverted)
{
    QImage image(cols, rows, QImage::Format_ARGB32);

    int tvalue = (inverted ? 0 : 255);
    int fvalue = (inverted ? 255 : 0);

    for (int h=0; h<rows; h++)
    {
        uchar * row = image.scanLine(h);
        for (int w=0; w<cols; w++)
        {
            uchar * px = row + (4*w);
            int test = (sl::binaryToGray(h+hoffset) & hmask) + (sl::binaryToGray(w+voffset) & vmask);
            int value = (test ? tvalue : fvalue);

            px[0] = value; //B
            px[1] = value; //G
            px[2] = value; //R
            px[3] = 0xff;  //A
        }
    }

    return QPixmap::fromImage(image);
}

cv::Mat rotateImage1(cv::Mat img, int degree, int mode = 1)
{
    degree = -degree;
    double angle = degree * CV_PI / 180.; // 弧度  
    double a = sin(angle), b = cos(angle);
    int width = img.cols;
    int height = img.rows;
    int width_rotate;
    int height_rotate;
    if (mode == 0)
    {
        width_rotate = int(pow(2, 0.5) * (width / 2 + height / 2) / 2);
        height_rotate = int(pow(2, 0.5) * (width / 2 + height / 2) / 2);
    }
    else if (mode == 1 || mode == 2)
    {
        width_rotate = int(height * fabs(a) + width * fabs(b));
        height_rotate = int(width * fabs(a) + height * fabs(b));
    }

    //旋转数组map
    // [ m0  m1  m2 ] ===>  [ A11  A12   b1 ]
    // [ m3  m4  m5 ] ===>  [ A21  A22   b2 ]
    float map[6];
    cv::Mat map_matrix = cv::Mat(2, 3, CV_32F, map);
    // 旋转中心
    cv::Point2f center = cv::Point2f(width / 2, height / 2);
    map_matrix = cv::getRotationMatrix2D(center, degree, 1.0);
    std::cout << (width_rotate - width) / 2 << " " << (height_rotate - height) / 2 << std::endl;
    std::cout << map_matrix << std::endl;
    map_matrix.at<double>(0, 2) += (width_rotate - width) / 2;
    map_matrix.at<double>(1, 2) += (height_rotate - height) / 2;
    std::cout << map_matrix << std::endl;
    cv::Mat img_rotate;
    //对图像做仿射变换
    //CV_WARP_FILL_OUTLIERS - 填充所有输出图像的象素。
    //如果部分象素落在输入图像的边界外，那么它们的值设定为 fillval.
    //CV_WARP_INVERSE_MAP - 指定 map_matrix 是输出图像到输入图像的反变换，
    cv::warpAffine(img, img_rotate, map_matrix, cv::Size(width_rotate, height_rotate), cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0);
    if (mode == 2)
    {
        cv::Range Range_rows = cv::Range(int(width * fabs(a)), height_rotate - int(width * fabs(a)));
        cv::Range Rangey_cols = cv::Range(int(height * fabs(a)), width_rotate - int(height * fabs(a)));

        img_rotate = img_rotate(Range_rows, Rangey_cols);
    }
    return img_rotate;
}


void ProjectorWidget::generate_sapce_pattern(int rows, int cols)
{

    int cubelength = 5;
    int RowSideCount = 16;
    int ColsSideCount = 32;
    cv::Mat CodeImage(2 * RowSideCount * cubelength, ColsSideCount * cubelength, CV_8UC1, cv::Scalar(0));
    //imshow("CodeImage1", CodeImage);


    for (int i = 0; i < RowSideCount; i++)
    {
        for (int rowOfCude = 0; rowOfCude < cubelength; rowOfCude++)
        {
            uchar* dataOfEachRow = CodeImage.ptr<uchar>(2 * i * cubelength + rowOfCude);
            for (int j = 0; j < ColsSideCount; j++)
            {
                uchar value;
                if (j % 2 == 1)
                {
                    value = 128;
                }
                else
                {
                    value = 0;
                }
                for (int k = 0; k < cubelength; k++)
                {
                    dataOfEachRow[j * cubelength + k] = value;
                }
            }
        }
        for (int rowOfCude = 0; rowOfCude < cubelength; rowOfCude++)
        {
            uchar* dataOfEachRow = CodeImage.ptr<uchar>((2 * i + 1) * cubelength + rowOfCude);
            for (int j = 0; j < ColsSideCount; j++)
            {
                uchar value;
                if (j % 2 == 1)
                {
                    value = 255;
                }
                else
                {
                    value = 125;
                }
                for (int k = 0; k < cubelength; k++)
                {
                    dataOfEachRow[j * cubelength + k] = value;
                }
            }
        }
        //for(int i)
    }
    cv::RotatedRect rRect = cv::RotatedRect(cv::Point2f(CodeImage.rows / 2, CodeImage.cols / 2), cv::Size2f(pow(2, 0.5) * 2 * RowSideCount * cubelength / 2, pow(2, 0.5) * ColsSideCount * cubelength / 2), 45); //定义一bai个旋转矩形
    cv::Point2f vertices[4];
    rRect.points(vertices);//提取旋转矩形的四个角点du
    for (int i = 0; i < 4; i++)
    {
        cv::line(CodeImage, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 255, 0));//四个角点连成线，最终形成旋转的矩形。
    }

    std::vector<bool> firstRow = { 1,1,0,0,1,0,0,1,1,0,1,0,0,0,1,1 };
    std::vector<bool> secondRow = { 1,1,0,0,1,1,1,1,0,0,0,1,0,1,0,0 };

    for (int i = 0; i < firstRow.size(); i++)
    {
        if (i % 2 == 0)
        {
            bool tem_value = firstRow[i];
            firstRow[i] = secondRow[i];
            secondRow[i] = tem_value;
        }
    }

    for (int i = RowSideCount - 1; i < 2 * RowSideCount; i++) //i-th row
    {
        int inedx_colum = i - (RowSideCount - 1);  //the index of the row colored now
        int count = 0;   //the number of colored cube in each line
        int start_row = i;
        int start_col = i - (RowSideCount - 1);

        while (count < 16)
        {
            int tem_value;

            if (inedx_colum % 2 == 0)
            {
                if (firstRow[count])
                {
                    tem_value = -75;
                }
                else
                {
                    tem_value = 75;
                }

            }
            else
            {
                if (secondRow[count])
                {
                    tem_value = -75;
                }
                else
                {
                    tem_value = 75;
                }
            }

            for (int k = 0; k < cubelength; k++)
            {
                uchar* DataOfEachRow = CodeImage.ptr<uchar>(start_row * cubelength + k);
                for (int num = start_col * cubelength; num < start_col * cubelength + cubelength; num++)
                {
                    DataOfEachRow[num] += tem_value;
                }
            }

            start_row--;
            start_col++;
            count++;
        }
    }

    //imshow("CodeImage2", CodeImage);

    //GaussianBlur(CodeImage, CodeImage, Size(5, 5), 10, 10);
    cv::blur(CodeImage, CodeImage, cv::Size(5, 5));
    //imshow("CodeImage3", CodeImage);
    CodeImage = rotateImage1(CodeImage, 45, 0);
    //imshow("CodeImage4", CodeImage);
    int Threshold = 120;
    for (int i = 0; i < CodeImage.rows; i++)
    {
        uchar* DataOfEachRow = CodeImage.ptr<uchar>(i);
        for (int j = 0; j < CodeImage.cols; j++)
        {
            if (DataOfEachRow[j] > Threshold)
            {
                DataOfEachRow[j] = 255;
            }
            else
            {
                DataOfEachRow[j] = 0;
            }
        }
    }
    //imshow("CodeImage5", CodeImage);

    int repeatrow = (int)(rows / CodeImage.rows) * 2;
    int repeatcol = (int)(cols / CodeImage.cols) * 2;
    cv::Mat RepeatCodeImage(CodeImage.rows * repeatrow, CodeImage.cols * repeatcol, CV_8UC1, cv::Scalar(0));
    for (int i = 0; i < repeatrow; i++)
    {
        for (int j = 0; j < repeatcol; j++)
        {
            for (int index_row = 0; index_row < CodeImage.rows; index_row++)
            {
                for (int index_col = 0; index_col < CodeImage.cols; index_col++)
                {
                    RepeatCodeImage.at<uchar>(i * CodeImage.rows + index_row, j * CodeImage.cols + index_col)
                        = CodeImage.at<uchar>(index_row, index_col);
                }
            }
        }
    }
    //imshow("CodeImage6", RepeatCodeImage);
    ////RepeatCodeImage = rotateImage1(RepeatCodeImage, 10, 1);

    //imshow("CodeImage7", rotateImage1(RepeatCodeImage, 10, 1));

    RepeatCodeImage = rotateImage1(RepeatCodeImage, 10, 2);
    cv::Range Range_rows = cv::Range(0, rows);
    cv::Range Range_cols = cv::Range(0, cols);
    this->_SpaceCodeImage = RepeatCodeImage(Range_rows, Range_cols).clone();
    imshow("Pattern", this->_SpaceCodeImage);

}

QPixmap ProjectorWidget::make_space_pattern(int rows, int cols, int inverted)
{
    if (this->_SpaceCodeImage.empty())
    {
        generate_sapce_pattern(rows, cols);
    }
    cv::Mat ShowPattern;
    QImage image = QImage(cols, rows, QImage::Format_Indexed8);
    if (inverted)
    {
        ShowPattern = 255 - this->_SpaceCodeImage;
        image = QImage((const unsigned char*)(ShowPattern.data), ShowPattern.cols, ShowPattern.rows, QImage::Format_Grayscale8);
    }
    else
    {
        ShowPattern = this->_SpaceCodeImage;
        image = QImage((const unsigned char*)(ShowPattern.data), ShowPattern.cols, ShowPattern.rows, QImage::Format_Grayscale8);
    }


    //gray_codes.clear();
    //Mat emptyCode(RepeatCodeImage.rows, RepeatCodeImage.cols, CV_8UC1, Scalar(0));
    //gray_codes.push_back(emptyCode);
    //gray_codes.push_back(RepeatCodeImage);

    //imshow("inverseCode", inverseCode);
    //gray_codes.push_back(inverseCode);
    return QPixmap::fromImage(image);
}



bool ProjectorWidget::save_info(QString const& filename, bool invert) const
{
    FILE * fp = fopen(qPrintable(filename), "w");
    if (!fp)
    {   //failed
        std::cerr << "Projector save_info failed, file: " << qPrintable(filename) << std::endl;
        return false;
    }

    int cols = width();
    int rows = height();

    if (invert)
    { //rotated image
      rows = width();
      cols  = height();
    }

    int effective_width = cols;
    int effective_height = rows;

    int max_vert_value = (1<<std::min(_vbits,_pattern_count));
    while (effective_width>max_vert_value )
    {
        effective_width >>= 1;
    }
    int max_horz_value = (1<<std::min(_hbits,_pattern_count));
    while (effective_height>max_horz_value)
    {
        effective_height >>= 1;
    }

    fprintf(fp, "%u %u\n", effective_width, effective_height);

    fprintf(fp, "\n# width height\n"); //help

    std::cerr << "Saved projetor info: " << qPrintable(filename) << std::endl
              << " - Effective resolution: " << effective_width << "x" << effective_height << std::endl;

    //close
    fclose(fp);
    return true;
}
