#include <iostream>
#include <fstream>

#include "imgui.h"
#include "app_povi.h"
#include "point_edge.h"
#include <array>

#include <boost/program_options.hpp>

// point_edge stuff
static config c;
static std::vector<bg::model::polygon<point_type>> footprints;
static bg::model::polygon<point_type> footprint;
static PNL_vector points;
static std::vector<PNL_vector> points_vec;
static std::vector<linedect::Point> edge_points;
static std::vector<std::pair<Point,Point>> edge_segments;
static Arrangement_2 arr;

// povi stuff
static double center_x;
static double center_y;
static std::vector<point_type> centroids;
static std::vector<GLfloat> segment_array;
static std::vector<GLfloat> polygon_array;
static auto pc_painter = std::make_shared<Painter>();
static auto fp_painter = std::make_shared<Painter>();
static auto steppoint_painter = std::make_shared<Painter>();
static auto segment_painter = std::make_shared<Painter>();
static auto polygon_painter = std::make_shared<Painter>();
// static std::weak_ptr<Painter> sp_handle, pp_handle, pc_handle, pce_handle, fp_handle;
static std::vector<GLfloat> point_array;
static std::vector<GLfloat> footprint_array;
static std::vector<GLfloat> edge_point_array;
static int footprint_id=0;
static float footprint_simp_thres=0;

static poviApp a(1280, 800, "Step edge detector");

void set_footprint(int index){
    footprint = footprints[index];
    points = points_vec[index];

    center_x = bg::get<0>(centroids[index]);
    center_y = bg::get<1>(centroids[index]);
    point_array.clear();
    point_array.resize(2*3*points.size());
    int i=0;
    for (auto &p : points) {
        point_array[i++] = p.get<0>().x() - center_x;
        point_array[i++] = p.get<0>().y() - center_y;
        point_array[i++] = p.get<0>().z();
        point_array[i++] = 0.6;
        point_array[i++] = 0.6;
        point_array[i++] = 0.6;
    }

    pc_painter->set_data(&point_array[0], point_array.size(), {3,3});
    footprint_array.clear();
    for (auto p : footprint.outer()) {
        footprint_array.push_back(bg::get<0>(p) - center_x);
        footprint_array.push_back(bg::get<1>(p) - center_y);
        footprint_array.push_back(0.0);
        footprint_array.push_back(0.0);
        footprint_array.push_back(1.0);
        footprint_array.push_back(0.0);
    }

    fp_painter->set_data(&footprint_array[0], footprint_array.size(), {3,3});
    edge_point_array.clear();

    steppoint_painter->set_data(&edge_point_array[0], edge_point_array.size(), {3,3});
    segment_array.clear();

    segment_painter->set_data(&segment_array[0], segment_array.size(), {3,3});
    polygon_array.clear();

    polygon_painter->set_data(&polygon_array[0], polygon_array.size(), {3,3});
    // a.center(bg::get<0>(centroids[index])-center_x, bg::get<1>(centroids[index])-center_y);
}
void compute_metrics(){
    compute_metrics(points, c);
}
void classify_edgepoints(){
    edge_points.clear();
    classify_edgepoints(edge_points, points, c);
    edge_point_array.clear();
    edge_point_array.resize(edge_points.size()*3*2);
    int i=0;
    for(auto p:edge_points){
        edge_point_array[i++] = p.x()-center_x;
        edge_point_array[i++] = p.y()-center_y;
        edge_point_array[i++] = p.z();
        edge_point_array[i++] = 0.57;
        edge_point_array[i++] = 0.81;
        edge_point_array[i++] = 0.94;
    }
    std::cout << "Found " << edge_points.size() << " edge points" << std::endl;

    steppoint_painter->set_data(&edge_point_array[0], edge_point_array.size(), {3,3});
}
void detect_lines(){
    edge_segments.clear();
    detect_lines(edge_segments, edge_points, c);
    segment_array.clear();
    segment_array.resize(edge_segments.size()*2*2*3);
    int i=0;
    for (auto s : edge_segments){
        segment_array[i++] = s.first.x()-center_x;
        segment_array[i++] = s.first.y()-center_y;
        segment_array[i++] = s.first.z();
        segment_array[i++] = 1.0;
        segment_array[i++] = 0.0;
        segment_array[i++] = 0.0;
        segment_array[i++] = s.second.x()-center_x;
        segment_array[i++] = s.second.y()-center_y;
        segment_array[i++] = s.second.z();
        segment_array[i++] = 1.0;
        segment_array[i++] = 0.0;
        segment_array[i++] = 0.0;
    }

    segment_painter->set_data(&segment_array[0], segment_array.size(), {3,3});
  
}
void build_arrangement(float simplification_thres){
    
    if(edge_segments.size()==0) return;
    polygon_array.clear();
    arr.clear();
    bg::model::polygon<point_type> bag_polygon, fp;
    bg::simplify(footprint, fp, simplification_thres);
    build_arrangement(fp, edge_segments, arr);
    for (auto face: arr.face_handles()){
        if(face->data()==1){
            auto he = face->outer_ccb();
            auto first = he;

            while(true){
                polygon_array.push_back(CGAL::to_double(he->source()->point().x())-center_x);
                polygon_array.push_back(CGAL::to_double(he->source()->point().y())-center_y);
                polygon_array.push_back(0);
                polygon_array.push_back(1);
                polygon_array.push_back(1);
                polygon_array.push_back(0);
                polygon_array.push_back(CGAL::to_double(he->target()->point().x())-center_x);
                polygon_array.push_back(CGAL::to_double(he->target()->point().y())-center_y);
                polygon_array.push_back(0);
                polygon_array.push_back(1);
                polygon_array.push_back(1);
                polygon_array.push_back(0);
                he = he->next();
                if (he==first) break;
            }
            polygon_array.push_back(CGAL::to_double(he->source()->point().x())-center_x);
            polygon_array.push_back(CGAL::to_double(he->source()->point().y())-center_y);
            polygon_array.push_back(0);
            polygon_array.push_back(1);
            polygon_array.push_back(1);
            polygon_array.push_back(0);
            polygon_array.push_back(CGAL::to_double(he->target()->point().x())-center_x);
            polygon_array.push_back(CGAL::to_double(he->target()->point().y())-center_y);
            polygon_array.push_back(0);
            polygon_array.push_back(1);
            polygon_array.push_back(1);
            polygon_array.push_back(0);
        }
    }

    polygon_painter->set_data(&polygon_array[0], polygon_array.size(), {3,3});
}

void write_arrangement(){
  std::ofstream f_arr("decomposed.wkt", std::ios::app);

  f_arr << std::fixed << std::setprecision(2);
  for (auto face: arr.face_handles()){
    if(face->data()==1){
      auto he = face->outer_ccb();
      auto first = he;

      f_arr << "POLYGON((";
      while(true){
        f_arr << he->source()->point().x() << " " << he->source()->point().y() << ",";
        he = he->next();
        if (he==first) break;
      }
      f_arr << he->source()->point().x() << " " << he->source()->point().y() << "))" << std::endl;
    }
  }
  f_arr.close();
}

void on_draw() {
    ImGui::Begin("Step edge parameters");
        if(footprints.size()>1){    
            if (ImGui::Button("Previous") && footprint_id-1 >= 0 &&  footprint_id-1 < footprints.size()){
                set_footprint(--footprint_id);
            }
            ImGui::SameLine();
            if (ImGui::Button("Next") && footprint_id+1 >= 0 && footprint_id+1 < footprints.size()){
                set_footprint(++footprint_id);
            }
            ImGui::PushItemWidth(200);
            if (ImGui::SliderInt("#", &footprint_id, 0, footprints.size()-1)) {
                set_footprint(footprint_id);
            }
            ImGui::PopItemWidth();
        }
        ImGui::PushItemWidth(100);
    // if (ImGui::CollapsingHeader("Compute metrics")){
        // ImGui::Indent();
        ImGui::InputInt("Plane min points", &c.metrics_plane_min_points);
        ImGui::InputFloat("Plane epsilon", &c.metrics_plane_epsilon, 0.01, 1);
        ImGui::InputFloat("Plane normal thres", &c.metrics_plane_normal_threshold, 0.01, 1);
        ImGui::InputFloat("Wall angle thres", &c.metrics_is_wall_threshold, 0.01, 1);
        ImGui::InputInt("K linefit", &c.metrics_k_linefit);
        ImGui::InputInt("K jumpedge", &c.metrics_k_jumpcnt_elediff);
        if (ImGui::Button("Compute"))
            compute_metrics();
        // ImGui::Unindent();
    // }
    // if (ImGui::CollapsingHeader("Classify edge points")){
        // ImGui::Indent();
        ImGui::InputInt("Jump cnt min", &c.classify_jump_count_min);
        ImGui::InputInt("Jump cnt max", &c.classify_jump_count_max);
        ImGui::InputFloat("Line dist", &c.classify_line_dist, 0.01, 1);
        ImGui::InputFloat("Elevation jump", &c.classify_jump_ele, 0.01, 1);
        if (ImGui::Button("Classify"))
            classify_edgepoints();
        // ImGui::Unindent();
    // }
    // if (ImGui::CollapsingHeader("Detect lines")){
        // ImGui::Indent();
        ImGui::InputFloat("Dist thres", &c.linedetect_dist_threshold, 0.01, 1);
        ImGui::InputInt("Segment cnt min", &c.linedetect_min_segment_count);
        ImGui::InputInt("K", &c.linedetect_k);
        if (ImGui::Button("Detect"))
            detect_lines();
        // ImGui::Unindent();
        ImGui::InputFloat("Footprint simp", &footprint_simp_thres, 0.01, 1);
        if (ImGui::Button("Build Arrangement"))
            build_arrangement(footprint_simp_thres);
        ImGui::SameLine();
        if (ImGui::Button("Write Arrangement"))
            write_arrangement();
        ImGui::PopItemWidth();
    // }
    ImGui::End();
}

namespace po = boost::program_options;
int main(int ac, const char * av[])
{
    std::string las_path = "/Users/ravi/surfdrive/data/step-edge-detector/ahn3.las";
    std::string csv_path = "/Users/ravi/surfdrive/data/step-edge-detector/rdam_sample_0.csv";
    std::string decomposed_path = "decomposed.wkt";
    
    po::options_description desc("Allowed options");
    desc.add_options()
    ("help", "produce help message")
    ("las", po::value<std::string>(&las_path), "Point cloud ")
    ("csv", po::value<std::string>(&csv_path), "Footprints ")
    // ("out", po::value<std::string>(&csv_path), "Decomposed footprints output")
    ;
    po::variables_map vm;
    po::store(po::parse_command_line(ac, av, desc), vm);
    po::notify(vm);
    if (vm.count("help")) {
        std::cout << desc << "\n";
        return 1;
    }

    std::cout << std::fixed << std::setprecision(2);
    
    auto csv_footprints = std::ifstream(csv_path);
    // auto csv_footprints = std::ifstream("/Users/ravi/surfdrive/data/step-edge-detector/bag_amersfoort_0.csv");
    std::string column_names, row;
    std::getline(csv_footprints, column_names);
    while (std::getline(csv_footprints, row)) {
        bg::model::polygon<point_type> bag_polygon;
        bg::read_wkt(row, bag_polygon);
        bg::unique(bag_polygon);
        footprints.push_back(bag_polygon);
    } csv_footprints.close();
    
    // bg::model::box<point_type> bbox;
    // bg::envelope(footprints[0], bbox);
    // double min_x = bg::get<bg::min_corner, 0>(bbox);
    // double min_y = bg::get<bg::min_corner, 1>(bbox);
    // double max_x = bbox.max_corner().get<0>();
    // double max_y = bbox.max_corner().get<1>();

    for (auto footprint:footprints){
        point_type p;
        bg::centroid(footprint, p);
        centroids.push_back(p);
    }
    pc_in_footprint(las_path, footprints, points_vec);
    // pc_in_footprint("/Users/ravi/surfdrive/data/step-edge-detector/C_31HZ1_clip.LAZ", footprints, points_vec);
    
    // prepare pointcloud painter
    pc_painter->set_data(&point_array[0], point_array.size(), {3,3});
    pc_painter->attach_shader("basic.vert");
    pc_painter->attach_shader("basic.frag");
    pc_painter->set_drawmode(GL_POINTS);
    // pc_painter->set_uniform("u_pointsize", 2.0);
    // prepare footprint painter
    fp_painter->set_data(&footprint_array[0], footprint_array.size(), {3,3});
    fp_painter->attach_shader("basic.vert");
    fp_painter->attach_shader("basic.frag");
    fp_painter->set_drawmode(GL_LINE_STRIP);
    // prepare step edge point painter
    steppoint_painter->set_data(&edge_point_array[0], edge_point_array.size(), {3,3});
    steppoint_painter->attach_shader("basic.vert");
    steppoint_painter->attach_shader("basic.frag");
    steppoint_painter->set_drawmode(GL_POINTS);
    // steppoint_painter->set_uniform("u_pointsize", 4.0);
    // prepare step edge segment painter
    segment_painter->set_data(&segment_array[0], segment_array.size(), {3,3});
    segment_painter->attach_shader("basic.vert");
    segment_painter->attach_shader("basic.frag");
    segment_painter->set_drawmode(GL_LINES);
    // prepare arrangement polygon painter
    polygon_painter->set_data(&segment_array[0], segment_array.size(), {3,3});
    polygon_painter->attach_shader("basic.vert");
    polygon_painter->attach_shader("basic.frag");
    polygon_painter->set_drawmode(GL_LINES);

    a.draw_that(on_draw);

    a.add_painter(pc_painter, "All pts");
    a.add_painter(fp_painter, "Footprint");
    a.add_painter(steppoint_painter, "Step pts");
    a.add_painter(segment_painter, "Step edges");
    a.add_painter(polygon_painter, "Decomposition");

    set_footprint(0);

    std::ofstream f_arr("decomposed.wkt");
    f_arr.close();

    a.run();
}
