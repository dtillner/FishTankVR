#include <QApplication>
#include <QLabel>
#include <QVector3D>
#include <QGLViewer/qglviewer.h>
#include <opencv2/opencv.hpp>

class FishTankVR : public QGLViewer {

    private:
        QImage* image;
        QLabel* label;
        QVector3D* facePosition;
        cv::VideoCapture* cam;
        cv::CascadeClassifier* frontFaceDetector;

    public:
        FishTankVR() {
            frontFaceDetector = new cv::CascadeClassifier("haarcascade_frontalface_default.xml");
            cam = new cv::VideoCapture(0);
            cv::Mat frame;
            *cam >> frame;
            facePosition = new QVector3D();
            image = new QImage(frame.cols, frame.rows, QImage::Format_RGB32);
            label = new QLabel(this);
            startTimer(100);
        }

        ~FishTankVR() {
            delete cam;
            delete image;
            delete label;
            delete frontFaceDetector;
        }

    private:
        void timerEvent(QTimerEvent*) {
            cv::Mat frame;
            cv::Mat monoFrame;
            cv::Mat smallMonoFrame;
            *cam >> frame;
            std::vector<cv::Rect> objects;

            cv::cvtColor(frame, monoFrame, CV_BGR2GRAY);
            cv::resize(monoFrame, smallMonoFrame, cv::Size(frame.cols/5, frame.rows/5));
            frontFaceDetector->detectMultiScale(smallMonoFrame, objects);

            if(objects.size() > 0) {
                objects[0].x *= 5;
                objects[0].y *= 5;
                objects[0].width *= 5;
                objects[0].height *= 5;
                cv::rectangle(frame, objects[0], cv::Scalar(255, 255, 255), 5);

                facePosition->setX(objects[0].x*5 + (objects[0].width*5) / 2);
                facePosition->setY(objects[0].y*5 + (objects[0].height*5) / 2);
            }

            convertImage(&frame);
            label->setPixmap(QPixmap::fromImage(image->scaled(width()/3, height()/3)));

            updateGL();
        }

        void resizeEvent(QResizeEvent* event) {
            label->setGeometry(width()-width()/3-10, height()-height()/3-10, width()/3, height()/3);
            QGLViewer::resizeEvent(event);
        }

        void convertImage(cv::Mat* cvImage) {
            int idx=0;
            int red, green, blue;

            for(int y=0; y<image->size().height(); ++y) {
                for(int x=0; x<image->size().width(); ++x) {
                    red = cvImage->data[idx+2];
                    green = cvImage->data[idx+1];
                    blue = cvImage->data[idx];
                    image->setPixel(x, y, qRgb(red, green, blue));
                    idx += 3;
                }
            }
        }

        void draw() {
            glPushMatrix();
                glTranslatef(0.75, 0, 0);
                glColor4f(1, 0, 0, 1);
                gluSphere(gluNewQuadric(), 0.5, 25, 25);
            glPopMatrix();

            glPushMatrix();
                glTranslatef(-0.75, 0.5, 0);
                glRotatef(90, 1, 0, 0);
                glColor4f(0, 0, 1, 1);
                gluCylinder(gluNewQuadric(), 0.5, 0.5, 1, 25, 1);
            glPopMatrix();

            camera()->setPosition(qglviewer::Vec::Vec(-(facePosition->x()-1000)/500, -(facePosition->y()-1000)/500, camera()->position().z));
            camera()->lookAt(qglviewer::Vec::Vec(0,0,0));

        }
};

int main(int argc, char *argv[]) {
    QApplication application(argc, argv);
    FishTankVR viewer;
    viewer.show();
    return application.exec();
}
