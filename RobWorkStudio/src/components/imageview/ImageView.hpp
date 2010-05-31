/*
 * ImageView.hpp
 *
 *  Created on: May 26, 2010
 *      Author: lpe
 */

#ifndef RWSTUDIO_COMPONENTS_IMAGEVIEW_HPP
#define RWSTUDIO_COMPONENTS_IMAGEVIEW_HPP

#include <QLabel>
#include <rw/sensor/Image.hpp>

class ImageView: public QLabel
{
    Q_OBJECT
public:
    ImageView(QWidget* parent = NULL);
    virtual ~ImageView();

    void display(const rw::sensor::Image& image);

};

#endif /* RWSTUDIO_COMPONENTS_IMAGEVIEW_HPP */
