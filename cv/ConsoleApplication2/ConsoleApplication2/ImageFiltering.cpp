// ImageFiltering.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <iostream>
#include <string>

using namespace std;
using namespace cv;

int B = 0;
int G = 1;
int R = 2;

int brightnessFactor;
int darknessFactor;
int redColor;
int greenColor;
int blueColor;
int tolerance = 50;

struct Color
{
	int minPrimary;
	int maxSecondary;
};

Color red,green,blue,yellow,black;

void setColorValues()
{
	red.minPrimary = 128;
	red.maxSecondary = 64;
	green.minPrimary = 64;
	green.maxSecondary = 155;
	blue.minPrimary = 64;
	blue.maxSecondary = 129;
	yellow.minPrimary = 150;
	yellow.maxSecondary = 129;
	black.minPrimary = 0;
	black.maxSecondary = 64;
}

int menu()
{
	int menuChoice;
	cout << endl << "What would you like to do?" << endl;
	cout << "1 - Filter image for red" << endl;
	cout << "2 - Filter image for green" << endl;
	cout << "3 - Filter image for blue" << endl;
	cout << "4 - Filter image for yellow" << endl;
	cout << "5 - Filter image for black" << endl;
	cout << "6 - Edit colors in image" << endl;
	cout << "7 - Brighten image" << endl;
	cout << "8 - Darken image" << endl;
	cout << "9 - Mirror image" << endl;
	cout << "10 - Filter image for R,G,B,Y" << endl;
	cout << "0 - Exit" << endl;
	cin >> menuChoice;
	
	return menuChoice;
}

string imageMenu()
{
	int imageChoice;
	string imageFileName;
	vector<string> fileList;
	cout << endl << "Which image to use?" << endl;
	cout << "1 - image1.jpg" << endl;
	cout << "2 - image2.jpg" << endl;
	cout << "3 - image3.jpg" << endl;
	cout << "4 - image4.jpg" << endl;
	cout << "5 - image5.jpg" << endl;
	cout << "6 - image6.jpg" << endl;
	cout << "7 - image7.jpg" << endl;
	cout << "8 - image8.jpg" << endl;
	cout << "0 - Exit" << endl;
	cin >> imageChoice;

	switch (imageChoice)
	{
		case 1: 
			imageFileName = "image1.jpg";
			break;
		case 2:
			imageFileName = "image2.jpg";
			break;
		case 3:
			imageFileName = "image3.jpg";
			break;
		case 4:
			imageFileName = "image4.jpg";
			break;
		case 5:
			imageFileName = "image5.jpg";
			break;
		case 6:
			imageFileName = "image6.jpg";
			break;
		case 7:
			imageFileName = "image7.jpg";
			break;
		case 8:
			imageFileName = "image8.jpg";
			break;
		case 0:
			break;
		default:
			cout << "Try Again" << endl;
			imageMenu();
			break;
	}
	cout << imageFileName << endl;
	return imageFileName;
}

Mat brightenImage(Mat image)
{
	cout << "Brighten image how much?" << endl;
	cin >> brightnessFactor;
	if (brightnessFactor < 255)
	{
		Mat imageHigh = image + Scalar(brightnessFactor, brightnessFactor, brightnessFactor);
		return imageHigh;
	}
	else
	{
		cout << "Unable to complete task" << endl;
		return image;
	}
}

Mat darkenImage(Mat image)
{
	cout << "Darken image how much?" << endl;
	cin >> darknessFactor;
	if (brightnessFactor < 255)
	{
		Mat imageLow = image - Scalar(darknessFactor, darknessFactor, darknessFactor);
		return imageLow;
	}
	else
	{
		cout << "Unable to complete task" << endl;
		return image;
	}
	
}

Mat editColors(Mat image)
{
	cout << "Edit Red color how much?" << endl;
	cin >> redColor;
	cout << "Edit Green color how much?" << endl;
	cin >> greenColor;
	cout << "Edit Blue color how much?" << endl;
	cin >> blueColor;
	image = image + Scalar(blueColor, greenColor, redColor);
	return image;
}

Mat mirrorImage(Mat image)
{
	for (int i = 0;i < (image.cols/2);i++)
	{
		for (int j = 0;j < image.rows;j++)
		{
			int tempRed;
			int tempGreen;
			int tempBlue;
			Vec3b &intensity = image.at<Vec3b>(j, i);
			Vec3b &intensity2 = image.at<Vec3b>(j, (image.cols)-i-1);
			tempRed = intensity.val[R];
			tempGreen = intensity.val[G];
			tempBlue = intensity.val[B];
			intensity.val[R] = intensity2.val[R];
			intensity.val[G] = intensity2.val[G];
			intensity.val[B] = intensity2.val[B];
			intensity2.val[R] = tempRed;
			intensity2.val[G] = tempGreen;
			intensity2.val[B] = tempBlue;
		}
	}
	return image;
}

Mat filterRed(Mat image)
{
	for (int i = 0;i < image.cols;i++)
	{
		for (int j = 0;j < image.rows;j++)
		{
			Vec3b &intensity = image.at<Vec3b>(j, i);
			if ((intensity.val[R] >= red.minPrimary) && (intensity.val[G] <= red.maxSecondary) && (intensity.val[B] <= red.maxSecondary))
			{
				intensity.val[R] = 255;
				intensity.val[G] = 0;
				intensity.val[B] = 0;
			}
			else
			{
				intensity.val[B] = 255;
				intensity.val[G] = 255;
				intensity.val[R] = 255;
			}
		}
	}
	return image;
}

Mat filterGreen(Mat image)
{
	for (int i = 0;i < image.cols;i++)
	{
		for (int j = 0;j < image.rows;j++)
		{
			Vec3b &intensity = image.at<Vec3b>(j, i);
			if ((intensity.val[G] >= green.minPrimary)  
				&& ((intensity.val[G] > intensity.val[R]) && (intensity.val[G] > intensity.val[B]))
				&& ((intensity.val[R] <= green.maxSecondary) && (intensity.val[B] <= green.maxSecondary)))
			{
				intensity.val[R] = 0;
				intensity.val[G] = 255;
				intensity.val[B] = 0;
			}
			else
			{
				intensity.val[B] = 255;
				intensity.val[G] = 255;
				intensity.val[R] = 255;
			}
		}
	}
	return image;
}

Mat filterBlue(Mat image)
{
	for (int i = 0;i < image.cols;i++)
	{
		for (int j = 0;j < image.rows;j++)
		{
			Vec3b &intensity = image.at<Vec3b>(j, i);
			if ((intensity.val[B] >= blue.minPrimary) && (intensity.val[R] <= blue.maxSecondary)  
				&& (intensity.val[B] > intensity.val[R]) && (intensity.val[B] >= intensity.val[G]))
			{
				intensity.val[R] = 0;
				intensity.val[G] = 0;
				intensity.val[B] = 255;
			}
			else
			{
				intensity.val[B] = 255;
				intensity.val[G] = 255;
				intensity.val[R] = 255;
			}
		}
	}
	return image;
}

Mat filterYellow(Mat image)
{
	for (int i = 0;i < image.cols;i++)
	{
		for (int j = 0;j < image.rows;j++)
		{
			Vec3b &intensity = image.at<Vec3b>(j, i);
			if ((intensity.val[R] >= yellow.minPrimary) && (intensity.val[G] >= yellow.minPrimary) && (intensity.val[B] <= yellow.maxSecondary)
				&& (abs(intensity.val[R]-intensity.val[G]) <= tolerance))
			{
				intensity.val[R] = 255;
				intensity.val[G] = 255;
				intensity.val[B] = 0;
			}
			else
			{
				intensity.val[B] = 255;
				intensity.val[G] = 255;
				intensity.val[R] = 255;
			}
		}
	}
	return image;
}

Mat filterBlack(Mat image)
{
	for (int i = 0;i < image.cols;i++)
	{
		for (int j = 0;j < image.rows;j++)
		{
			Vec3b &intensity = image.at<Vec3b>(j, i);
			if ((intensity.val[R] <= black.maxSecondary) && (intensity.val[G] <= black.maxSecondary) && (intensity.val[B] <= black.maxSecondary))
			{
				intensity.val[R] = 0;
				intensity.val[G] = 0;
				intensity.val[B] = 0;
			}
			else
			{
				intensity.val[B] = 255;
				intensity.val[G] = 255;
				intensity.val[R] = 255;
			}
		}
	}
	return image;
}

Mat filterRGBY(Mat image)
{
	int redCount = 0;
	int greenCount = 0;
	int blueCount = 0;
	int yellowCount = 0;
	for (int i = 0;i < image.cols;i++)
	{
		for (int j = 0;j < image.rows;j++)
		{
			Vec3b &intensity = image.at<Vec3b>(j, i);
			if ((intensity.val[R] >= red.minPrimary) && (intensity.val[G] <= red.maxSecondary) && (intensity.val[B] <= red.maxSecondary))
			{
				intensity.val[R] = 255;
				intensity.val[G] = 0;
				intensity.val[B] = 0;
				redCount = redCount + 1;
			}
			else if ((intensity.val[G] >= green.minPrimary)
				&& ((intensity.val[G] > intensity.val[R]) && (intensity.val[G] > intensity.val[B]))
				&& ((intensity.val[R] <= green.maxSecondary) && (intensity.val[B] <= green.maxSecondary)))
			{
				intensity.val[R] = 0;
				intensity.val[G] = 255;
				intensity.val[B] = 0;
				greenCount = greenCount + 1;
			}
			else if ((intensity.val[B] >= blue.minPrimary) && (intensity.val[R] <= blue.maxSecondary)
				&& (intensity.val[B] > intensity.val[R]) && (intensity.val[B] >= intensity.val[G]))
			{
				intensity.val[R] = 0;
				intensity.val[G] = 0;
				intensity.val[B] = 255;
				blueCount = blueCount + 1;
			}
			else if ((intensity.val[R] >= yellow.minPrimary) && (intensity.val[G] >= yellow.minPrimary) && (intensity.val[B] <= yellow.maxSecondary)
				&& (abs(intensity.val[R] - intensity.val[G]) <= tolerance))
			{
				intensity.val[R] = 255;
				intensity.val[G] = 255;
				intensity.val[B] = 0;
				yellowCount = yellowCount + 1;
			}
			else
			{
				intensity.val[B] = 255;
				intensity.val[G] = 255;
				intensity.val[R] = 255;
			}
		}
	}
	if ((redCount > greenCount) && (redCount > blueCount) && (redCount > yellowCount))
	{
		cout << "Red Object" << endl;
	}
	else if ((greenCount > redCount) && (greenCount > blueCount) && (greenCount > yellowCount))
	{
		cout << "Green Object" << endl;
	}
	else if ((blueCount > redCount) && (blueCount > greenCount) && (blueCount > yellowCount))
	{
		cout << "Blue Object" << endl;
	}
	else if ((yellowCount > redCount) && (yellowCount > greenCount) && (yellowCount > blueCount))
	{
		cout << "Yellow Object" << endl;
	}
	else
	{
		cout << "Don't Know" << endl;
	}
	return image;
}

int main(int argc, const char** argv)
{
	setColorValues();
	string imageChoice;
	int menuChoice = 11;
	while (menuChoice != 0)
	{
		imageChoice = imageMenu();
		menuChoice = menu();
		if (menuChoice > 0)
		{
			
			Mat image = imread(imageChoice, CV_LOAD_IMAGE_COLOR);
			Mat imageOriginal = imread(imageChoice, CV_LOAD_IMAGE_COLOR);

			if (image.empty()) //check whether the image is loaded or not
			{
				cout << "Error : Image cannot be loaded..!!" << endl;
				system("pause"); //wait for a key press
				return -1;
			}
			
	
			//cout << image.channels() << endl;

			switch (menuChoice)
			{
				case 1: 
					image = filterRed(image);
					break;
				case 2: 
					image = filterGreen(image);
					break;
				case 3: 
					image = filterBlue(image);
					break;
				case 4:
					image = filterYellow(image);
					break;
				case 5:
					image = filterBlack(image);
					break;
				case 6:
					image = editColors(image);
					break;
				case 7:
					image = brightenImage(image);
					break;
				case 8:
					image = darkenImage(image);
					break;
				case 9:
					image = mirrorImage(image);
					break;
				case 10:
					image = filterRGBY(image);
					break;
				default: 
					image = image;
					break;
			}
			

			namedWindow("Original Image", CV_WINDOW_NORMAL);
			imshow("Original Image", imageOriginal);
			

			namedWindow("Altered Image", CV_WINDOW_NORMAL);
			imshow("Altered Image", image);
			waitKey(0); //wait infinite time for a keypress

			destroyWindow("Original Image"); //destroy the window with the name, "MyWindow"
			destroyWindow("Altered Image"); //destroy the window with the name, "MyWindow"

			/*vector<int> compression_params;
			compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
			compression_params.push_back(50);
			cout << "Made it here" << endl;
			try {
				imwrite("alteredImage.jpg", image, compression_params);
			}
			catch (runtime_error& ex) {
				fprintf(stderr, "Exception converting image to JPG format: %s\n", ex.what());
				return 1;
			}

			fprintf(stdout, "Saved JPG file\n");
			
			image.deallocate();
			imageOriginal.deallocate();*/
			
		}
	}
	return 0;
}



	
