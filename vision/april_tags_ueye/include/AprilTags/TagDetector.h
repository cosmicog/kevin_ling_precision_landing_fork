#ifndef TAGDETECTOR_H
#define TAGDETECTOR_H

#include <vector>

#include "opencv2/opencv.hpp"

#include "AprilTags/TagDetection.h"
#include "AprilTags/TagFamily.h"
#include "AprilTags/FloatImage.h"

namespace AprilTags {

class TagDetector {
public:
	
	const TagFamily thisTagFamily;

	//! Constructor
  // note: TagFamily is instantiated here from TagCodes
	TagDetector(const TagCodes& tagCodes) : thisTagFamily(tagCodes) {}
	
	std::vector<TagDetection> extractTagsOriginal(const cv::Mat& image);
	std::vector<TagDetection> extractTags(const cv::Mat& image, cv::Mat* roi, int offset_x, int offset_y);
	
};

} // namespace

#endif
