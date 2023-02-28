#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/io_handlers/Factory.h>
#include <lanelet2_io/io_handlers/Writer.h>
#include <lanelet2_projection/UTM.h>

#include <cstdio>

#include <opencv2/opencv.hpp>

#pragma GCC diagnostic ignored "-Wunused-but-set-variable"

// we want assert statements to work in release mode
#undef NDEBUG

namespace {
std::string exampleMapPath = std::string(PKG_DIR) + "/../lanelet2_maps/res/lanelet2_map.osm";

std::string tempfile(const std::string& name) {
  char tmpDir[] = "/tmp/lanelet2_example_XXXXXX";
  auto* file = mkdtemp(tmpDir);
  if (file == nullptr) {
    throw lanelet::IOError("Failed to open a temporary file for writing");
  }
  return std::string(file) + '/' + name;
}

cv::Mat image = cv::Mat::zeros(cv::Size(4000,4000), CV_8UC3);
cv::Mat h_ref_mat = cv::Mat::zeros(cv::Size(1,1), CV_8UC3);
cv::Mat bgr_color_mat = cv::Mat::zeros(cv::Size(1,1), CV_8UC3);
}  // namespace

void part1LoadingAndWriting();
void part2Projectors();
void part3AddingNewParsersAndWriters();

int main() {
  // this tutorial shows you how to load and write lanelet maps. It is divided into three parts:
  part1LoadingAndWriting();
  part2Projectors();
  part3AddingNewParsersAndWriters();
  return 0;
}

void drawCenterline(cv::Mat image, std::vector< std::pair< double, lanelet::ConstLanelet > > lanelets, int scale, std::vector<int> offset){
  using namespace lanelet;
  for(auto lanelet: lanelets){
    
    const auto centerline = lanelet.second.invert().centerline();
    const auto dataptr =  centerline.constData();
    auto data = *dataptr;
    const auto points = data.points();
    int tmp_x = 0;
    int tmp_y = 0;

    for(auto coordinates: points){
        //std::cout<<coordinates<<std::endl;
        int x = int(scale*(coordinates.x()+offset[0]));
        int y = int(scale*(coordinates.y()+offset[1]));
        int angle = std::atan2(double(y-tmp_y), double(x-tmp_x))*360/(2*M_PI)+180;

        if(tmp_x!=0){
          cv::Vec<unsigned char,3> hsv_color{(unsigned char)(angle/2), 255, 255};
          h_ref_mat.at<cv::Vec3b>(0,0) = hsv_color;
          cv::cvtColor(h_ref_mat, bgr_color_mat, cv::COLOR_HSV2BGR);
          auto line_color = bgr_color_mat.at<cv::Vec3b>(0,0);
          cv::line(image, cv::Point(tmp_x,tmp_y),cv::Point(x,y), line_color, 20 ,cv::LINE_AA);
        }
        tmp_x = x;
        tmp_y = y;
    }
  }

}

void part1LoadingAndWriting() {
  using namespace lanelet;
  // loading a map requires two things: the path and either an origin or a projector that does the lat/lon->x/y
  // conversion.
  projection::UtmProjector projector(Origin({35.6, 139.7}));  // we will go into details later
  LaneletMapConstPtr map = load(exampleMapPath, projector);
  const lanelet::BasicPoint2d search_point(0,0);
  // nearest lanelet
  int scale = 40;
  std::vector<int> offset = {156400, -60480};
  constexpr double search_radius = 100.0;  // [m]
  const auto surrounding_lanelets = 
  lanelet::geometry::findNearest(map->laneletLayer, search_point, search_radius);
  int counter=0;
  for(auto lanelet: surrounding_lanelets){
    //get points of center line
    const auto centerline = lanelet.second.centerline();
    const auto dataptr =  centerline.constData();
    auto data = *dataptr;
    const auto points = data.points();

    //get points of left line
    const auto left_bound = lanelet.second.leftBound().invert();
    const auto left_bound_dataptr = left_bound.constData();
    auto left_bound_data = *left_bound_dataptr;
    const auto left_bound_points = left_bound_data.points();

    //get points of right line
    const auto right_bound = lanelet.second.rightBound();
    const auto right_bound_dataptr = right_bound.constData();
    auto right_bound_data = *right_bound_dataptr;
    const auto right_bound_points = right_bound_data.points();

    // std::vector<lanelet::Point3d> bound_points;
    // bound_points.insert(bound_points.end(), left_bound_points.begin(), left_bound_points.end()); // bound_points.insert(bound_points.end(), right_bound_points.begin(), right_bound_points.end());
    std::vector<std::vector<cv::Point2i>> lane_area;
    std::vector<cv::Point2i> left_lane;
    std::vector<cv::Point2i> right_lane;
    std::vector<cv::Point2i> center_lane;
    cv::Point2i tmp_point;

    for(auto center_point : points){
      tmp_point = {int(scale*(center_point.x()+offset[0])), int(scale*(center_point.y()+offset[1]))};
      center_lane.push_back(tmp_point);
    }

    for(auto bound_point : left_bound_points){
      tmp_point = {int(scale*(bound_point.x()+offset[0])), int(scale*(bound_point.y()+offset[1]))};
      left_lane.push_back(tmp_point);
    }

    for(auto bound_point : right_bound_points){
      tmp_point = {int(scale*(bound_point.x()+offset[0])), int(scale*(bound_point.y()+offset[1]))};
      right_lane.push_back(tmp_point);
    }

    std::reverse(left_lane.begin(), left_lane.end());
    std::reverse(right_lane.begin(), right_lane.end());
    left_lane.insert(left_lane.end(), center_lane.begin(), center_lane.end());
    //left_lane.insert(left_lane.end(), right_lane.begin(), right_lane.end());
    right_lane.insert(right_lane.end(), center_lane.begin(), center_lane.end());


    // std::cout<<"left:"<<left_lane<<std::endl;
    // std::cout<<"right:"<<right_lane<<std::endl;
    // std::cout<<"center:"<<center_lane<<std::endl;
    lane_area.push_back(left_lane);
    lane_area.push_back(right_lane);
    cv::fillPoly(image, lane_area, cv::Scalar(127,127,127), cv::LINE_AA);
    // for(auto point:left_lane){
    //   cv::circle(image, point, 5 ,cv::Scalar(0,5*counter,0), cv::FILLED);
    // }
    // for(auto point:right_lane){
    //   cv::circle(image, point, 8 ,cv::Scalar(5*counter,0,0), cv::FILLED);
    // }
    std::cout<<"num of lanelets: " <<left_lane.size()+right_lane.size() <<std::endl;
    counter++;

  }
  
  drawCenterline(image, surrounding_lanelets, scale, offset);
  std::cout<<"all_lanelet_elements: "<<map->size() <<std::endl;
  cv::imwrite("test.jpg", image);
  // the load and write functions are agnostic to the file extension. Depending on the extension, a different loading
  // algorithm will be chosen. Here we chose osm.

  // we can also load and write into an internal binary format. It is not human readable but loading is much faster
  // than from .osm:
  write(tempfile("map_t4.bin"), *map);  // we do not need a projector to write to bin

  // if the map could not be parsed, exceptoins are thrown. Alternatively, you can provide an error struct. Then
  // lanelet2 will load the map as far as possible and write all the errors that occured to the error object that you
  // passed:
  ErrorMessages errors;
  map = load(exampleMapPath, projector, &errors);
  assert(errors.empty());  // of no errors occurred, the map could be fully parsed.
}

void part2Projectors() {
  using namespace lanelet;
  // as mentioned, projectors do the lat/lon->x/y conversion. This conversion is not trivial and only works when an
  // origin close to the actual map position is chosen. Otherwise the loaded map will be distorted.
  projection::UtmProjector projector(Origin({35.6, 139.7}));
  BasicPoint3d projection = projector.forward(GPSPoint{35.6, 139.7, 0});
  assert(std::abs(projection.x()) < 1e-6);

  // by default, lanele2 picks a projector that implements the mercator projection. However this is only due to legacy
  // reasons and because it is cheap and efficient to implement. In general, we recommend to use the UTM projector (we
  // already used it above). if you load a map from osm without providing a suitable projector or origin, an exception
  // will be thrown.
  // LaneletMapPtr map = load(exampleMapPath); // throws: loading from osm without projector
}

// you can easily add new parsers and writers so that they will be picked by load/write.
// here we write a writer that simply does nothing. We will not write a reader, but it works similarly.
namespace example {
class FakeWriter : public lanelet::io_handlers::Writer {
 public:
  using Writer::Writer;
  void write(const std::string& /*filename*/, const lanelet::LaneletMap& /*laneletMap*/,
             lanelet::ErrorMessages& /*errors*/, const lanelet::io::Configuration& /*params*/) const override {
    // this writer does just nothing
  }
  static constexpr const char* extension() {
    return ".fake";  // this is the extension that we support
  }

  static constexpr const char* name() {
    return "fake_writer";  // this is the name of the writer. Users can also pick the writer by its name.
  }
};
}  // namespace example

namespace {
// this registers our new class for lanelet2_io
lanelet::io_handlers::RegisterWriter<example::FakeWriter> reg;
}  // namespace

void part3AddingNewParsersAndWriters() {
  using namespace lanelet;
  // now we can test our writer:
  LaneletMap map;
  write("anypath.fake", map);
}
