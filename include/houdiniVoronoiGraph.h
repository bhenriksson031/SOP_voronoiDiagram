#include <unordered_map>

#include <vector>


#include <GU/GU_Detail.h>
#include <GA/GA_Detail.h>
#include <GA/GA_Iterator.h>
#include <OP/OP_Error.h>
#include <GA/GA_Stat.h>
#include <GEO/GEO_PrimPoly.h>

#include <boost/polygon/voronoi.hpp>
#include <boost/polygon/polygon.hpp>



using boost::polygon::voronoi_builder;
using boost::polygon::voronoi_diagram;
using boost::polygon::x;
using boost::polygon::y;
using boost::polygon::low;
using boost::polygon::high;

#include "voronoi_visual_utils.hpp"


//TODO find intermediate format
struct Point {
	double a;
	double b;
	Point(double x, double y) : a(x), b(y) {}
	Point() : a(0), b(0) {}
	double x() { return a ; }
	double y() { return b; }
	double x(double val) {a = val;}
	double y(double val) {b = val;}
	double set(double x, double y) { a = x; b = y; }

};

struct Segment {
	Point p0;
	Point p1;
	Segment(double x1, double y1, double x2, double y2) : p0(x1, y1), p1(x2, y2) {}
};


namespace boost {
	namespace polygon {

		template <>
		struct geometry_concept<Point> {
			typedef point_concept type;
		};

		template <>
		struct point_traits<Point> {
			typedef double coordinate_type;

			static inline coordinate_type get(
				const Point& point, orientation_2d orient) {
				return (orient == HORIZONTAL) ? point.a : point.b;
			}
		};
		/*
		//modified from point_traits.hpp
		template <>
		struct point_mutable_traits<Point> {
			typedef double coordinate_type;

			static void set(
				Point& point, orientation_2d orient, coordinate_type value) {
				point.set(orient, value);
			}

			static Point construct(coordinate_type x, coordinate_type y) {
				return Point(x, y);
			}
		};
		*/
		template <>
		struct geometry_concept<Segment> {
			typedef segment_concept type;
		};

		template <>
		struct segment_traits<Segment> {
			typedef double coordinate_type;
			typedef Point point_type;

			static inline point_type get(const Segment& segment, direction_1d dir) {
				return dir.to_int() ? segment.p1 : segment.p0;
			}
		};
	}  // polygon
}  // boost

using namespace boost::polygon;
		class boostVoronoiGraph {
		

		private:
			typedef double coordinate_type;
			typedef point_data<coordinate_type> point_type;
			typedef segment_data<coordinate_type> segment_type;
			typedef rectangle_data<coordinate_type> rect_type;
			typedef voronoi_builder<int> VB;  //int from https://www.boost.org/doc/libs/1_71_0/libs/polygon/example/voronoi_visualizer.cpp
			typedef voronoi_diagram<coordinate_type> VD;
			typedef VD::cell_type cell_type;
			typedef VD::cell_type::source_index_type source_index_type;
			typedef VD::cell_type::source_category_type source_category_type;
			typedef VD::edge_type edge_type;
			typedef VD::cell_container_type cell_container_type;
			typedef VD::cell_container_type vertex_container_type;
			typedef VD::edge_container_type edge_container_type;
			typedef VD::const_cell_iterator const_cell_iterator;
			typedef VD::const_vertex_iterator const_vertex_iterator;
			typedef VD::const_edge_iterator const_edge_iterator;


			GA_Offset addPointFromVDPt(GU_Detail *gdp, point_type p) {
				GA_Offset ptoff = gdp->appendPoint();
				UT_Vector3 PValue = UT_Vector3(x(p), 0.0f, y(p)); //TODO get edge point position
				gdp->setPos3(ptoff, PValue);
				return ptoff;
			}


			int houMeshLoader(GU_Detail *gdp) {
				// Preparing Input Geometries.
				//TODO add all gdp positions to pts
				printf("houMeshLoader...\n");
				point_data_.push_back(point_type(0.0, 0.0));
				point_data_.push_back(point_type(1.0, 6.0));
				segment_data_.push_back(segment_type(point_type(-4.0, 5.0), point_type(5.0, -1.0) ) );
				segment_data_.push_back(segment_type(point_type(3.0, -11.0), point_type(13.0, -1.0)));

				update_brect(point_type(13.0, 6.0));
				update_brect(point_type(-4.0, -11.0));

				return(0);
			}
			//reference code
			/*
			int HouMeshLoader(GU_Detail *gdp, UT_BoundingBox &bbox)
			{
				double scale = bbox.sizeMax()*0.5;
				if (scale < .00001) {
					std::cout << "scale is zero!";
					return(14);
				}
				this->normalize_scale = scale;
				this->normalize_offset = Vector3d(bbox.centerX(), bbox.centerY(), bbox.centerZ());
				GA_RWHandleV3 Phandle(gdp->findAttribute(GA_ATTRIB_POINT, "P"));
				GA_Offset ptoff;
				GEO_Primitive *prim;
				int i = 0;
				//set size of matrices
				GA_Range ptrange = gdp->getPointRange();
				GA_Range primrange = gdp->getPrimitiveRange();
				GA_Size npts = gdp->getPointRange().getEntries();
				GA_Size nprims = primrange.getEntries();

				V.resize(3, npts);
				F.resize(3, nprims);
			_
			 -+///std::cout<<"matrix V size: "<< V.size() << "\n";
				//std::cout << "matrix F size: " << F.size() <<"\n";
				GA_FOR_ALL_PTOFF(gdp, ptoff) {
					UT_Vector3 Pvalue = (Phandle.get(ptoff) - bbox.center()) / scale;
					V(0, i) = Pvalue.x();
					V(1, i) = Pvalue.y();
					V(2, i) = Pvalue.z();
					i++;
				}

				int j = 0;
				GA_FOR_ALL_PRIMITIVES(gdp, prim) {

					GA_OffsetListRef prim_vrts = gdp->getPrimitiveVertexList(prim->getMapOffset());

					if (prim_vrts.entries() != 3) {
						return(14); //Bad input code
					}
					F(0, j) = gdp->vertexPoint(prim_vrts(0));
					F(1, j) = gdp->vertexPoint(prim_vrts(1));
					F(2, j) = gdp->vertexPoint(prim_vrts(2));
					j++;
				}
				return(0);
			}
			*/
			void discretize(
				const point_type& point,
				const segment_type& segment,
				const double max_dist,
				std::vector< point_type >* discretization) {
				printf("discretize...\n");
				// Apply the linear transformation to move start point of the segment to
				// the point with coordinates (0, 0) and the direction of the segment to
				// coincide the positive direction of the x-axis.
				double segm_vec_x = static_cast<double>(x(high(segment))) - static_cast<double>(x(low(segment)));
				double segm_vec_y = static_cast<double>(y(high(segment))) - static_cast<double>(y(low(segment)));
				double sqr_segment_length = segm_vec_x * segm_vec_x + segm_vec_y * segm_vec_y;

				// Compute x-coordinates of the endpoints of the edge
				// in the transformed space.
				double projection_start = sqr_segment_length *
					get_point_projection((*discretization)[0], segment);
				double projection_end = sqr_segment_length *
					get_point_projection((*discretization)[1], segment);

				// Compute parabola parameters in the transformed space.
				// Parabola ´/*-+
				// Compute parabola parameters in the transformed space.
				// Parabola has next representation:
				// f(x) = ((x-rot_x)^2 + rot_y^2) / (2.0*rot_y).
				double point_vec_x = static_cast<double>(x(point)) - static_cast<double>(x(low(segment)));
				double point_vec_y = static_cast<double>(y(point)) - static_cast<double>(y(low(segment)));
				double rot_x = segm_vec_x * point_vec_x + segm_vec_y * point_vec_y;
				double rot_y = segm_vec_x * point_vec_y - segm_vec_y * point_vec_x;

				// Save the last point.
				point_type last_point = (*discretization)[1];
				discretization->pop_back();

				// Use stack to avoid recursion.
				std::stack<double> point_stack;
				point_stack.push(projection_end);
				double cur_x = projection_start;
				double cur_y = parabola_y(cur_x, rot_x, rot_y);

				// Adjust max_dist parameter in the transformed space.
				const double max_dist_transformed = max_dist * max_dist * sqr_segment_length;
				while (!point_stack.empty()) {
					double new_x = point_stack.top();
					double new_y = parabola_y(new_x, rot_x, rot_y);

					// Compute coordinates of the point of the parabola that is
					// furthest from the current line segment.
					double mid_x = (new_y - cur_y) / (new_x - cur_x) * rot_y + rot_x;
					double mid_y = parabola_y(mid_x, rot_x, rot_y);

					// Compute maximum distance between the given parabolic arc
					// and line segment that discretize it.
					double dist = (new_y - cur_y) * (mid_x - cur_x) -
						(new_x - cur_x) * (mid_y - cur_y);
					dist = dist * dist / ((new_y - cur_y) * (new_y - cur_y) +
						(new_x - cur_x) * (new_x - cur_x));
					if (dist <= max_dist_transformed) {
						// Distance between parabola and line segment is less than max_dist.
						point_stack.pop();
						double inter_x = (segm_vec_x * new_x - segm_vec_y * new_y) /
							sqr_segment_length + static_cast<double>(x(low(segment)));
						double inter_y = (segm_vec_x * new_y + segm_vec_y * new_x) /
							sqr_segment_length + static_cast<double>(y(low(segment)));
						discretization->push_back(point_type(inter_x, inter_y));
						cur_x = new_x;
						cur_y = new_y;
					}
					else {
						point_stack.push(mid_x);
					}
				}

				// Update last point.
				discretization->back() = last_point;
			}


		 // Compute y(x) = ((x - a) * (x - a) + b * b) / (2 * b).
		 static double parabola_y(double x, double a, double b) {
			 return ((x - a) * (x - a) + b * b) / (b + b);
		 }

		 // Get normalized length of the distance between:
		 //   1) point projection onto the segment
		 //   2) start point of the segment
		 // Return this length divided by the segment length. This is made to avoid
		 // sqrt computation during transformation from the initial space to the
		 // transformed one and vice versa. The assumption is made that projection of
		 // the point lies between the start-point and endpoint of the segment.

		 double get_point_projection(
				 const point_type& point, const segment_type& segment) {
			 double segment_vec_x = static_cast<double>(x(high(segment))) - static_cast<double>(x(low(segment)));
			 double segment_vec_y = static_cast<double>(y(high(segment))) - static_cast<double>(y(low(segment)));
			 double point_vec_x = x(point) - static_cast<double>(x(low(segment)));
			 double point_vec_y = y(point) - static_cast<double>(y(low(segment)));
			 double sqr_segment_length =
				 segment_vec_x * segment_vec_x + segment_vec_y * segment_vec_y;
			 double vec_dot = segment_vec_x * point_vec_x + segment_vec_y * point_vec_y;
			 return vec_dot / sqr_segment_length;
		 }
				segment_type retrieve_segment(const cell_type& cell) {
					source_index_type index = cell.source_index() - point_data_.size();
					return segment_data_[index];
				}




			point_type retrieve_point(const voronoi_diagram<coordinate_type>::cell_type& cell) {
				source_index_type index = cell.source_index();
				source_category_type category = cell.source_category();
				if (category == boost::polygon::SOURCE_CATEGORY_SINGLE_POINT) {
					return point_data_[index];
				}
				index -= point_data_.size(); //TODO get input points, should have been implemented in top class to be accessible globally but simpler solution is to pass points and segments to addVoronoiDiagramToHouMesh
				if (category == boost::polygon::SOURCE_CATEGORY_SEGMENT_START_POINT) {
					return low(segment_data_[index]);
				}
				else {
					return high(segment_data_[index]);
				}
			}

			void clip_infinite_edge(
				const edge_type& edge, std::vector<point_type>* clipped_edge) {
				printf("clip_infinite_edge...\n");
				const cell_type& cell1 = *edge.cell();
				const cell_type& cell2 = *edge.twin()->cell();
				point_type origin, direction;
				// Infinite edges could not be created by two segment sites.
				if (cell1.contains_point() && cell2.contains_point()) {
					point_type p1 = retrieve_point(cell1);
					point_type p2 = retrieve_point(cell2);
					origin.x((p1.x() + p2.x()) * 0.5);
					origin.y((p1.y() + p2.y()) * 0.5);
					direction.x(p1.y() - p2.y());
					direction.y(p2.x() - p1.x());
				}
				else {
					origin = cell1.contains_segment() ?
						retrieve_point(cell2) :
						retrieve_point(cell1);
					segment_type segment = cell1.contains_segment() ?
						retrieve_segment(cell1) :
						retrieve_segment(cell2);
					coordinate_type dx = high(segment).x() - low(segment).x();
					coordinate_type dy = high(segment).y() - low(segment).y();
					if ((low(segment).x()==0.0 &&  low(segment).y() == origin.x() ) ^ cell1.contains_point()) {
						direction.x(dy);
						direction.y(-dx);
					}
					else {
						direction.x(-dy);
						direction.y(dx);
					}
				}
				coordinate_type side = xh(brect_) - xl(brect_);
				coordinate_type koef =
					side / (std::max)(fabs(direction.x()), fabs(direction.y()));
				if (edge.vertex0() == NULL) {
					clipped_edge->push_back(point_type(
						origin.x() - direction.x() * koef,
						origin.y() - direction.y() * koef));
				}
				else {
					clipped_edge->push_back(
						point_type(edge.vertex0()->x(), edge.vertex0()->y()));
				}
				if (edge.vertex1() == NULL) {
					clipped_edge->push_back(point_type(
						origin.x() + direction.x() * koef,
						origin.y() + direction.y() * koef));
				}
				else {
					clipped_edge->push_back(
						point_type(edge.vertex1()->x(), edge.vertex1()->y()));
				}
			}
			void sample_curved_edge(const voronoi_diagram<coordinate_type>::edge_type& edge, std::vector<point_type>* sampled_edge) {
				printf("sample_curved_edge...\n");
				double max_dist = .1; // 1E-3 * (xh(brect_) - xl(brect_));
				point_type point = edge.cell()->contains_point() ? retrieve_point(*edge.cell()) : retrieve_point(*edge.twin()->cell());
				segment_type segment = edge.cell()->contains_point() ?
					retrieve_segment(*edge.twin()->cell()) :
					retrieve_segment(*edge.cell());
				discretize(point, segment, max_dist, sampled_edge);
			}

			// Traversing Voronoi edges using cell iterator.
			int iterate_primary_edges2(GU_Detail *gdp, const voronoi_diagram<coordinate_type> &vd, std::vector<point_type>& points, std::vector<segment_type>& segments) {
				printf("iterate_primary_edges2...\n");
				int result = 0;
				GA_Offset ptoff;
				for (voronoi_diagram<coordinate_type>::const_cell_iterator it = vd.cells().begin();
					it != vd.cells().end(); ++it) {
					const voronoi_diagram<coordinate_type>::cell_type& cell = *it;
					const voronoi_diagram<coordinate_type>::edge_type* edge = cell.incident_edge();
					// This is convenient way to iterate edges around Voronoi cell.
					GEO_PrimPoly *poly = (GEO_PrimPoly *)gdp->appendPrimitive(GA_PRIMPOLY);
					do {
						if (edge->is_primary()) {
							std::vector<point_type> samples;
							if (!edge->is_finite()) {
								clip_infinite_edge(*edge, &samples);
							}
							else {
								point_type vertex0(edge->vertex0()->x(), edge->vertex0()->y());
								samples.push_back(vertex0);
								point_type vertex1(edge->vertex1()->x(), edge->vertex1()->y());
								samples.push_back(vertex1);
								if (edge->is_curved()) {
									sample_curved_edge(*edge, &samples);
								}
							}
							for (int i = 0; i < samples.size(); ++i) {
								point_type p0 = samples[i];
								ptoff = addPointFromVDPt(gdp, p0);
								poly->appendVertex(ptoff);
							}
							const voronoi_diagram<coordinate_type>::vertex_type *vtx0 = edge->vertex0();
							const voronoi_diagram<coordinate_type>::vertex_type *vtx1 = edge->vertex1();
							if (vtx0 != nullptr && vtx1 != nullptr) {
								double x0 = vtx0->x();  //TODO printing test chrashes Hou
								double y0 = vtx0->y();
								double x1 = vtx1->x();  //TODO printing test chrashes Hou
								double y1 = vtx1->y();
								printf("Found points: %f, %f, %f, %f\n", x0, y0, x1, y1);
								point_type p0 = point_type(x0, y0);
								point_type p1 = point_type(x1, y1);
								//add point for vertex position
								ptoff = addPointFromVDPt(gdp, p0);
								poly->appendVertex(ptoff);
								ptoff = addPointFromVDPt(gdp, p1);
								poly->appendVertex(ptoff);
							}

							++result;
						}
						edge = edge->next();
					} while (edge != cell.incident_edge());
				}
				return result;
			}

			// Traversing Voronoi edges using vertex iterator.
			// As opposite to the above two functions this one will not iterate through
			// edges without finite endpoints and will iterate only once through edges
			// with single finite endpoint.
			int iterate_primary_edges3(GU_Detail *gdp, const voronoi_diagram<coordinate_type> &vd) {
				int result = 0;
				//TODO add polygon
				GA_Offset ptoff;
				for (voronoi_diagram<coordinate_type>::const_vertex_iterator it =
					vd.vertices().begin(); it != vd.vertices().end(); ++it) {
					const voronoi_diagram<coordinate_type>::vertex_type& vertex = *it;
					const voronoi_diagram<coordinate_type>::edge_type* edge = vertex.incident_edge();
					// This is convenient way to iterate edges around Voronoi vertex.
					GEO_PrimPoly *poly = (GEO_PrimPoly *)gdp->appendPrimitive(GA_PRIMPOLY);
					do {
						if (edge->is_primary()) {
							//add point for vertex position
							ptoff = addPointFromVDPt(gdp, point_type(vertex.x(), vertex.y()));
							poly->appendVertex(ptoff);
							//add vertex on polygon for point
							++result;
						}
						edge = edge->rot_next();
					} while (edge != vertex.incident_edge());
				}
				return result;
			}



			int addVoronoiDiagramToHouMesh(GU_Detail *gdp, voronoi_diagram<coordinate_type>& vd, std::vector<point_type>& points, std::vector<segment_type>& segments) {
				//TODO add voronoi cells as prims and points
				printf("addVoronoiDiagramToHouMesh...\n");
				//create geometry for voronoi graph
				{
					// Traversing Voronoi Graph.
					{
						printf("Traversing Voronoi graph.\n");
						//printf("Number of visited primary edges using edge iterator: %d\n",
							//iterate_primary_edges1(vd));
						//printf("Number of visited primary edges using cell iterator: %d\n",
							//iterate_primary_edges2(vd));
						//printf("Number of visited primary edges using vertex iterator: %d\n"),
							//iterate_primary_edges3(vd));
						printf("\n");
					}
				}
				iterate_primary_edges2(gdp, vd, points, segments);


				unsigned int cell_index = 0;
				GA_Offset ptoff;
				for (voronoi_diagram<coordinate_type>::const_cell_iterator it = vd.cells().begin();
					it != vd.cells().end(); ++it) {
					// This is convenient way to iterate edges around Voronoi vertex.
					if (it->contains_point()) {
						if (it->source_category() ==
							boost::polygon::SOURCE_CATEGORY_SINGLE_POINT) {
							std::size_t index = it->source_index();
							point_type p = points[index];
							//add hou pt from vd pt
							addPointFromVDPt(gdp, p);

						}
						else if (it->source_category() ==
							boost::polygon::SOURCE_CATEGORY_SEGMENT_START_POINT) {
							std::size_t index = it->source_index() - points.size();
							point_type p0 = low(segments[index]);
							ptoff = addPointFromVDPt(gdp, p0);
						}
						else if (it->source_category() ==
							boost::polygon::SOURCE_CATEGORY_SEGMENT_END_POINT) {
							std::size_t index = it->source_index() - points.size();
							point_type p1 = high(segments[index]);

							ptoff = addPointFromVDPt(gdp, p1);
						}
					}
					else {

						std::size_t index = it->source_index() - points.size();
						//TODO create polygon for edge
						point_type pt0 = low(segments[index]);
						point_type pt1 = high(segments[index]);
						GEO_PrimPoly *poly = (GEO_PrimPoly *)gdp->appendPrimitive(GA_PRIMPOLY);
						ptoff = addPointFromVDPt(gdp, pt0);
						poly->appendVertex(ptoff);
						ptoff = addPointFromVDPt(gdp, pt1);
						poly->appendVertex(ptoff);
					}
					++cell_index;
				}

				return(0);
			}
			void clear() {
				brect_initialized_ = false;
				point_data_.clear();
				segment_data_.clear();
				vd_.clear();
			}

			void update_brect(const point_type& point) {
				printf("update_brect...\n");
				if (brect_initialized_) {
					encompass(brect_, point);
				}
				else {
					set_points(brect_, point, point);
					brect_initialized_ = true;
				}

			}

			void construct_brect() {
				double side = (std::max)(xh(brect_) - xl(brect_), yh(brect_) - yl(brect_));
				center(shift_, brect_);
				set_points(brect_, shift_, shift_);
				bloat(brect_, side * 1.2);
			}
			public:
			//TODO draw voronoi graph
			void addVoronoiGraphToHoudiniGeo(GU_Detail *gdp) {
				clear();
				int test = houMeshLoader(gdp);
				construct_brect();
				printf("addVoronoiGraphToHoudiniGeo...\n");
				//TODO init intermediate format

				
				// Construction of the Voronoi Diagram.
				voronoi_diagram<coordinate_type> vd;
				construct_voronoi(point_data_.begin(), point_data_.end(),
					segment_data_.begin(), segment_data_.end(),
					&vd);

				addVoronoiDiagramToHouMesh(gdp, vd, point_data_, segment_data_);
			}

			//class data
			point_type shift_ = point_type(0.0, 0.0);
			rect_type brect_;
			std::vector<point_type> point_data_; 
			std::vector<segment_type> segment_data_;
			VB vb_;
			VD vd_;
			bool brect_initialized_;
			bool primary_edges_only_;
			bool internal_edges_only_;
			/*
			//old code here as ref
			void HouMeshDumper(GU_Detail *gdp) {
				//create all points
				std::vector<GA_Offset> added_pts;
				UT_Vector3 offs = UT_Vector3(this->normalize_offset.x(), this->normalize_offset.y(), this->normalize_offset.z());
				for (int i = 0; i < O_compact.size(); ++i) {

					GA_Offset ptoff = gdp->appendPoint();
					added_pts.push_back(ptoff);

					UT_Vector3 PValue = UT_Vector3(O_compact[i].x(), O_compact[i].y(), O_compact[i].z()) * this->normalize_scale + offs;
					gdp->setPos3(ptoff, PValue);
				}

				for (int i = 0; i < F_compact.size(); ++i)
				{

					GA_Offset pt0 = added_pts[F_compact[i][0]];
					GA_Offset pt1 = added_pts[F_compact[i][1]];
					GA_Offset pt2 = added_pts[F_compact[i][2]];
					GA_Offset pt3 = added_pts[F_compact[i][3]];
					GEO_PrimPoly *poly = (GEO_PrimPoly *)gdp->appendPrimitive(GA_PRIMPOLY);
					poly->appendVertex(pt0);
					poly->appendVertex(pt1);
					poly->appendVertex(pt2);
					poly->appendVertex(pt3);
					poly->close();
				}
				*/
		};
