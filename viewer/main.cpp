#include <iostream>

#include "imgui.h"
#include "app_povi.h"
#include "point_edge.h"
#include <array>

static config c;
static bg::model::polygon<point_type> footprint;
static PNL_vector points;
static std::vector<linedect::Point> edge_points;
static std::vector<std::pair<Point,Point>> edge_segments;
static poviApp a(1280, 800, "Step edge detector");

void compute_metrics(){
    compute_metrics(points, c);
}
void classify_edgepoints(){
    edge_points.clear();
    classify_edgepoints(edge_points, points, c);
}
void detect_lines(){
    edge_segments.clear();
    detect_lines(edge_segments, edge_points, c);
}

void on_draw() {
    ImGui::Begin("Step edge parameters");
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
        ImGui::InputFloat("input float", &c.classify_line_dist, 0.01, 1);
        ImGui::InputFloat("input float", &c.classify_jump_ele, 0.01, 1);
        if (ImGui::Button("Classify"))
            classify_edgepoints();
        // ImGui::Unindent();
    // }
    // if (ImGui::CollapsingHeader("Detect lines")){
        // ImGui::Indent();
        ImGui::InputFloat("input float", &c.linedetect_dist_threshold, 0.01, 1);
        ImGui::InputInt("Segment cnt min", &c.linedetect_min_segment_count);
        ImGui::InputInt("K", &c.linedetect_k);
        if (ImGui::Button("Detect"))
            detect_lines();
        // ImGui::Unindent();
    // }
    ImGui::End();
}

int main(void)
{
    std::cout << std::fixed << std::setprecision(2);
  
    bg::read_wkt("Polygon ((91716.13000000000465661 438312.07000000000698492, 91716.77199999999720603 438312.76799999998183921, 91717.66099999999278225 438313.73499999998603016, 91729.11999999999534339 438326.19000000000232831, 91729.2559999999939464 438326.0659999999916181, 91738.30999999999767169 438317.84000000002561137, 91739.07000000000698492 438318.66999999998370185, 91743.60000000000582077 438314.53999999997904524, 91749.55000000000291038 438321.03000000002793968, 91736.07000000000698492 438333.34000000002561137, 91731.43499999999767169 438337.55499999999301508, 91728.17699999999604188 438340.51799999998183921, 91727.96300000000337604 438340.71299999998882413, 91726.35000000000582077 438342.17999999999301508, 91730.4419999999954598 438346.66800000000512227, 91731.20600000000558794 438347.50500000000465661, 91743.52000000000407454 438361.01000000000931323, 91739.02800000000570435 438365.08699999999953434, 91736.75800000000162981 438367.14699999999720603, 91736.58999999999650754 438367.29999999998835847, 91722.71600000000034925 438352.11300000001210719, 91721.10899999999674037 438353.64000000001396984, 91711.05000000000291038 438342.59999999997671694, 91712.69500000000698492 438341.14600000000791624, 91709.50999999999476131 438337.65999999997438863, 91716.41400000000430737 438331.41300000000046566, 91717.96199999999953434 438330.01099999999860302, 91719.08000000000174623 438329, 91710.89999999999417923 438320.01000000000931323, 91707.22999999999592546 438323.28999999997904524, 91706.2559999999939464 438322.22399999998742715, 91704.65200000000186265 438320.46799999999348074, 91703.85000000000582077 438319.59000000002561137, 91703.91199999999662396 438319.53299999999580905, 91706.33999999999650754 438317.30999999999767169, 91693.41999999999825377 438303.19000000000232831, 91691 438305.40000000002328306, 91687.58999999999650754 438301.70000000001164153, 91697.96899999999732245 438292.19599999999627471, 91702.67799999999988358 438297.37099999998463318, 91707.28299999999580905 438302.4469999999855645, 91711.53299999999580905 438307.13299999997252598, 91711.66000000000349246 438307.28999999997904524, 91713.7470000000030268 438309.60700000001816079, 91716.13000000000465661 438312.07000000000698492))", footprint);
    
    bg::model::box<point_type> bbox;
    bg::envelope(footprint, bbox);
    double min_x = bg::get<bg::min_corner, 0>(bbox);
    double min_y = bg::get<bg::min_corner, 1>(bbox);
    double max_x = bbox.max_corner().get<0>();
    double max_y = bbox.max_corner().get<1>();
    double center_x = (min_x+max_x)/2;
    double center_y = (min_y+max_y)/2;

    
    pc_in_footprint("/Users/ravi/surfdrive/data/step-edge-detector/ahn3.las", footprint, points);

    // prepare pointcloud painter
    std::vector<GLfloat> point_array;
    point_array.resize(2*3*points.size());
    int i=0;
    for (auto &p : points) {
        point_array[i++] = p.get<0>().x() - center_x;
        point_array[i++] = p.get<0>().y() - center_y;
        point_array[i++] = p.get<0>().z();
        point_array[i++] = 1.0;
        point_array[i++] = 1.0;
        point_array[i++] = 1.0;
    }
    auto pc_painter = std::make_shared<Painter>();
    pc_painter->set_data(&point_array[0], point_array.size(), {3,3});
    pc_painter->attach_shader("basic.vert");
    pc_painter->attach_shader("basic.frag");
    pc_painter->set_drawmode(GL_POINTS);

    // prepare footprint painter
    std::vector<GLfloat> footprint_array;
    footprint_array.resize(2*3*(bg::num_points(footprint)));
    i=0;
    for (auto p : footprint.outer()) {
        footprint_array[i++] = bg::get<0>(p) - center_x;
        footprint_array[i++] = bg::get<1>(p) - center_y;
        footprint_array[i++] = 0.0;
        footprint_array[i++] = 0.0;
        footprint_array[i++] = 1.0;
        footprint_array[i++] = 0.0;
    }
    auto fp_painter = std::make_shared<Painter>();
    fp_painter->set_data(&footprint_array[0], footprint_array.size(), {3,3});
    fp_painter->attach_shader("basic.vert");
    fp_painter->attach_shader("basic.frag");
    fp_painter->set_drawmode(GL_LINE_STRIP);

    a.draw_that(on_draw);

    a.add_painter(std::move(pc_painter), "point cloud");
    a.add_painter(std::move(fp_painter), "footprint");

    a.run();
}
