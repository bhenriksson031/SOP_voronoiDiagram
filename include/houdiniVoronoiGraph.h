#include <unordered_map>

#include <vector>


#include <GU/GU_Detail.h>
#include <GA/GA_Detail.h>
#include <GA/GA_Iterator.h>
#include <OP/OP_Error.h>
#include <GA/GA_Stat.h>
#include <GEO/GEO_PrimPoly.h>

#include <boost/polygon/voronoi.hpp>
using boost::polygon::voronoi_builder;
using boost::polygon::voronoi_diagram;
using boost::polygon::x;
using boost::polygon::y;
using boost::polygon::low;
using boost::polygon::high;

#include "voronoi_visual_utils.hpp"

//#include "voronoi_visual_utils.hpp"


//TODO find intermediate format
struct Point {
	double a;
	double b;
	Point(double x, double y) : a(x), b(y) {}
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


//TODO convert mesh to boost.polygon 2d format
namespace boostVoronoiGraph {
	/*
	from voronoi visualizer, might have been better to implement this 
private:
	typedef double coordinate_type;
	typedef point_data<coordinate_type> point_type;
	typedef segment_data<coordinate_type> segment_type;
	typedef rectangle_data<coordinate_type> rect_type;
	typedef voronoi_builder<int> VB;
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
	*/
	//duplicate of above
//private:
	typedef double coordinate_type;
	typedef Point point_type;
	typedef Segment segment_type;
	//typedef boost::polygon::rectangle_data<double> rect_type;
	typedef voronoi_builder<double> VB;
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

	GA_Offset addPointFromVDPt(GU_Detail *gdp, Point p) {
		GA_Offset ptoff = gdp->appendPoint();
		UT_Vector3 PValue = UT_Vector3(x(p), 0.0f, y(p)); //TODO get edge point position
		gdp->setPos3(ptoff, PValue);
		return ptoff;
	}


	int houMeshLoader(GU_Detail *gdp, std::vector<Segment>& segments, std::vector<Point>& points) {
		// Preparing Input Geometries.
		//TODO add all gdp positions to pts
		printf("houMeshLoader...\n");
		points.push_back(Point(0.0, 0.0));
		points.push_back(Point(1.0, 6.0));
		segments.push_back(Segment(-4.0, 5.0, 5.0, -1.0));
		segments.push_back(Segment(3.0, -11.0, 13.0, -1.0));
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
		//std::cout<<"matrix V size: "<< V.size() << "\n";
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

	Segment retrieve_segment(const cell_type& cell, std::vector<Point>& points, std::vector<Segment>& segments) {
		source_index_type index = cell.source_index() - points.size();
		return segments[index];
	}



	Point retrieve_point(const voronoi_diagram<double>::cell_type& cell,  std::vector<Point>& points, std::vector<Segment>& segments) {
		source_index_type index = cell.source_index();
		source_category_type category = cell.source_category();
		if (category == boost::polygon::SOURCE_CATEGORY_SINGLE_POINT) {
			return points[index];
		}
		index -= points.size(); //TODO get input points, should have been implemented in top class to be accessible globally but simpler solution is to pass points and segments to addVoronoiDiagramToHouMesh
		if (category == boost::polygon::SOURCE_CATEGORY_SEGMENT_START_POINT) {
			return low(segments[index]);
		}
		else {
			return high(segments[index]);
		}
	}

	void sample_curved_edge( const voronoi_diagram<coordinate_type>::edge_type& edge, std::vector<Point>* sampled_edge, std::vector<Point>& points, std::vector<Segment>& segments) {
		double max_dist = .1; // 1E-3 * (xh(brect_) - xl(brect_));
		Point point = edge.cell()->contains_point() ? retrieve_point(*edge.cell(), points, segments) : retrieve_point(*edge.twin()->cell(), points, segments);
		segment_type segment = edge.cell()->contains_point() ?
		retrieve_segment(*edge.twin()->cell(), points, segments) :
		retrieve_segment(*edge.cell(), points, segments);
		boost::polygon::voronoi_visual_utils<coordinate_type>::discretize(point, segment, max_dist, sampled_edge);
	}

	// Traversing Voronoi edges using cell iterator.
	int iterate_primary_edges2(GU_Detail *gdp, const voronoi_diagram<double> &vd, std::vector<Point>& points, std::vector<Segment>& segments) {
		int result = 0;
		GA_Offset ptoff;
		for (voronoi_diagram<double>::const_cell_iterator it = vd.cells().begin();
			it != vd.cells().end(); ++it) {
			const voronoi_diagram<double>::cell_type& cell = *it;
			const voronoi_diagram<double>::edge_type* edge = cell.incident_edge();
			// This is convenient way to iterate edges around Voronoi cell.
			GEO_PrimPoly *poly = (GEO_PrimPoly *)gdp->appendPrimitive(GA_PRIMPOLY);
			do {
				if (edge->is_primary()) {
					if (edge->is_curved()) {
							std::vector<point_type> samples;
							sample_curved_edge(*edge, &samples, points, segments);
							for (int i = 0; i < samples.size(); ++i) {
								Point p0 = samples[i];
								ptoff = addPointFromVDPt(gdp, p0);
								poly->appendVertex(ptoff);
							}
						}
					const voronoi_diagram<double>::vertex_type *vtx0 = edge->vertex0();
					const voronoi_diagram<double>::vertex_type *vtx1 = edge->vertex1();
					if (vtx0 != nullptr && vtx1 != nullptr) {
						double x0 = vtx0->x();  //TODO printing test chrashes Hou
						double y0 = vtx0->y();
						double x1 = vtx1->x();  //TODO printing test chrashes Hou
						double y1 = vtx1->y();
						printf("Found points: %f, %f, %f, %f\n", x0, y0, x1, y1);
						Point p0 = Point(x0, y0);
						Point p1 = Point(x1, y1);
						//add point for vertex position
						ptoff = addPointFromVDPt(gdp, p0);
						poly->appendVertex(ptoff);
						ptoff =addPointFromVDPt(gdp, p1);
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
	int iterate_primary_edges3(GU_Detail *gdp, const voronoi_diagram<double> &vd) {
		int result = 0;
		//TODO add polygon
		GA_Offset ptoff;
		for (voronoi_diagram<double>::const_vertex_iterator it =
			vd.vertices().begin(); it != vd.vertices().end(); ++it) {
			const voronoi_diagram<double>::vertex_type& vertex = *it;
			const voronoi_diagram<double>::edge_type* edge = vertex.incident_edge();
			// This is convenient way to iterate edges around Voronoi vertex.
			GEO_PrimPoly *poly = (GEO_PrimPoly *)gdp->appendPrimitive(GA_PRIMPOLY);
			do {
				if (edge->is_primary()) {
					//add point for vertex position
					ptoff = addPointFromVDPt(gdp, Point(vertex.x(), vertex.y()));
					poly->appendVertex(ptoff);
					//add vertex on polygon for point
					++result;
				}
				edge = edge->rot_next();
			} while (edge != vertex.incident_edge());
		}
		return result;
	}



	int addVoronoiDiagramToHouMesh(GU_Detail *gdp, voronoi_diagram<double>& vd, std::vector<Point>& points, std::vector<Segment>& segments) {
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
		for (voronoi_diagram<double>::const_cell_iterator it = vd.cells().begin();
			it != vd.cells().end(); ++it) {
			// This is convenient way to iterate edges around Voronoi vertex.
			if (it->contains_point()) {
				if (it->source_category() ==
					boost::polygon::SOURCE_CATEGORY_SINGLE_POINT) {
					std::size_t index = it->source_index();
					Point p = points[index];
					//add hou pt from vd pt
					addPointFromVDPt(gdp, p);

				}
				else if (it->source_category() ==
					boost::polygon::SOURCE_CATEGORY_SEGMENT_START_POINT) {
					std::size_t index = it->source_index() - points.size();
					Point p0 = low(segments[index]);
					ptoff = addPointFromVDPt(gdp, p0);
				}
				else if (it->source_category() ==
					boost::polygon::SOURCE_CATEGORY_SEGMENT_END_POINT) {
					std::size_t index = it->source_index() - points.size();
					Point p1 = high(segments[index]);

					ptoff = addPointFromVDPt(gdp, p1);
				}
			}
			else {

				std::size_t index = it->source_index() - points.size();
				//TODO create polygon for edge
				Point pt0 = low(segments[index]);
				Point pt1 = high(segments[index]);
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


	//TODO draw voronoi graph
	void addVoronoiGraphToHoudiniGeo(GU_Detail *gdp) {
		printf("addVoronoiGraphToHoudiniGeo...\n");
		//TODO init intermediate format

		std::vector<Point> added_pts;
		std::vector<Segment> added_segments;

		int test = houMeshLoader(gdp, added_segments, added_pts);
		// Construction of the Voronoi Diagram.
		voronoi_diagram<double> vd;
		construct_voronoi(added_pts.begin(), added_pts.end(),
			added_segments.begin(), added_segments.end(),
			&vd);

		addVoronoiDiagramToHouMesh(gdp, vd, added_pts, added_segments);
	}
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
}