/*
 * ImageView.cpp
 *
 *  Created on: May 26, 2010
 *      Author: lpe
 */

#include "ImageView.hpp"

#include <QImage>
#include <QPixmap>

#include <iostream>

using namespace rw::sensor;

ImageView::ImageView(QWidget* parent):
    QLabel(parent)
{


}

ImageView::~ImageView()
{
}



void ImageView::display(const rw::sensor::Image& image) {
    QImage qimage(image.getWidth(), image.getHeight(), QImage::Format_RGB32);
    for (size_t i = 0; i<image.getWidth(); i++) {
        for (size_t j = 0; j<image.getHeight(); j++) {
            Pixel4f pixel = image.getPixel(i,j);
            int value = 0;
            value += 0xff000000;//((int)(255*pixel.ch[3]))<<24;
            value += ((int)(/*255.0**/pixel.ch[0]))<<16;
            value += ((int)(/*255.0**/pixel.ch[1]))<<8;
            value += (int)/*255.0**/pixel.ch[2];

            qimage.setPixel(i,j, value);
        }
    }

    QPixmap pixmap = QPixmap::fromImage(qimage);
    this->setPixmap(pixmap);

}
