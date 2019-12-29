#include <unordered_map>

#include <vector>


#include <GU/GU_Detail.h>
#include <GA/GA_Detail.h>
#include <GA/GA_Iterator.h>
#include <OP/OP_Error.h>
#include <GA/GA_Stat.h>

#include <GEO/GEO_Face.h>
#include <GEO/GEO_PrimPoly.h>
#include <GEO/GEO_AdjPolyIterator.h>

#include <boost/polygon/voronoi.hpp>
#include <boost/polygon/polygon.hpp>

using boost::polygon::voronoi_builder;
using boost::polygon::voronoi_diagram;
using boost::polygon::x;
using boost::polygon::y;
using boost::polygon::low;
using boost::polygon::high;

#include "voronoi_visual_utils.hpp"

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
			//TODO add vertex attributes
			GA_Offset ptoff = gdp->appendPoint();
			UT_Vector3 PValue = UT_Vector3(x(p)/ scale_up_factor_, 0.0f, y(p)/ scale_up_factor_); //TODO get edge point position
			gdp->setPos3(ptoff, PValue);
			temp_ptnum++;
			return ptoff;
		}			
			
		void addSegment(segment_type segment,const int prim_index){
			seg_parent_prims_.push_back(prim_index);
			segment_data_.push_back(segment);
		}

		void addPoint(point_type pt) {
			point_data_.push_back(pt);
		}

		void compute_size_factor(GU_Detail *gpd) {
			UT_BoundingBox bbox;
			gpd->computeQuickBounds(bbox);
			double bbox_size = bbox.sizeMax();
			scale_up_factor_ = max_coord_size_ / bbox_size;
			if(verbosity>0) printf("scale factor = %g \n", scale_up_factor_);
		}
			
		bool is_point_outside_brect(point_type &p) {
			if ( (p.x() < xl(brect_) || p.x() > xh(brect_) )|| (p.y() < yl(brect_) || p.y() > yh(brect_))) { return true; }
			return false;
		}

		void houMeshLoader(GU_Detail *gpd) {
			// Preparing Input Geometries.
			if (verbosity > 0)printf("houMeshLoader...\n");
			//iterate over prims 
			GEO_Primitive *prim;
			GA_Range vtx_range;
			int prim_index = 0;

			//add single points
			GA_Offset ptoff;
			GA_OffsetList unused_pts;
			bool found_unused = gpd->findUnusedPoints(&unused_pts);
			for(GA_Offset ptoff : unused_pts){
				UT_Vector3 offs = gpd->getPos3(ptoff);
				point_type pt_single = point_type(offs.x()* scale_up_factor_, offs.z() * scale_up_factor_);
				addPoint(pt_single);
				update_brect(pt_single);
			}

			GA_FOR_ALL_PRIMITIVES(gpd, prim){
				if (prim->getTypeId() == GEO_PRIMPOLY) {
					int i = 0;
					vtx_range = prim ->getVertexRange();
					std::vector<point_type> face_pts;
					point_type pt0, pt_start;
					for (GA_Iterator vtx_it(vtx_range.begin()); !vtx_it.atEnd(); ++vtx_it)
					{
						if (verbosity > 0)printf("adding point..\n");
						GA_Offset vtx_offset1 = vtx_it.getOffset();
						UT_Vector3 offs = gpd->getPos3(gpd->vertexPoint(vtx_offset1) );
						point_type pt1 = point_type(offs.x()* scale_up_factor_, offs.z() * scale_up_factor_);
						if (i == 0) pt_start = pt1;
						if (i>0) {
							if (verbosity > 0)printf("adding point..\n");
							addSegment(segment_type(pt0, pt1), prim_index);
							update_brect(pt1);
							if (i == 1) update_brect(pt0);
						}
						pt0 = point_type(offs.x() *scale_up_factor_ , offs.z() *scale_up_factor_);
						i++;
					}
					//add segment for closed prims
					GA_LocalIntrinsic intr(prim->findIntrinsic("closed"));
					int is_closed;
					prim->getIntrinsic(intr, is_closed);
					if (is_closed) {
						addSegment(segment_type(pt0, pt_start), prim_index); //last and first point
					}
					prim_index++;
				}
			}
			return;
		}

		void discretize(
			const point_type& point,
			const segment_type& segment,
			const double max_dist,
			std::vector< point_type >* discretization) {
			if (verbosity > 0)printf("discretize...\n");
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
			if (isnan(projection_end) || isnan(projection_start)) {
				if (verbosity > 1)printf("found nan on point projection, skipping discretization\n");
				return;
			}
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
			int i = 0;
			while (!point_stack.empty() && i<10000) {
				i++; //TODO implement safety catch
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
				
				//if (i == 10000)printf("too many points, dist= %g, max_dist_transformed = %g, new_y = %g, cur_y= %g, new_x=%g, rot_x= %g, rot_y= %g, point_vec_x = %g, point_vec_x \n", dist, max_dist_transformed, new_y, cur_y, new_x, rot_x, rot_y);
				//if (i == 10000)printf("point.x =%g, point.y =%g, point_vec_x =%g, point_vec_y =%g, projection_start= %g, projection_end= %g \n", point.x(), point.y(), point_vec_x, point_vec_y, projection_start, projection_end);
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

		point_type get_point_projection_point(
		const point_type& point, const segment_type& segment) {
		double segment_vec_x = static_cast<double>(x(high(segment))) - static_cast<double>(x(low(segment)));
		double segment_vec_y = static_cast<double>(y(high(segment))) - static_cast<double>(y(low(segment)));
		double point_vec_x = x(point) - static_cast<double>(x(low(segment)));
		double point_vec_y = y(point) - static_cast<double>(y(low(segment)));
		double sqr_segment_length =
			segment_vec_x * segment_vec_x + segment_vec_y * segment_vec_y;
		double vec_dot = segment_vec_x * point_vec_x + segment_vec_y * point_vec_y;

		double seg_parm = vec_dot / sqr_segment_length;
		point_type p_proj;
		p_proj.x(static_cast<double>(x(low(segment)) + segment_vec_x* seg_parm) );
		p_proj.y(static_cast<double>(y(low(segment)) + segment_vec_y* seg_parm) );
		return p_proj;
		}

		bool edge_is_internal_to_primid(const edge_type& edge) {
			const cell_type& cell1 = *edge.cell();
			const cell_type& cell2 = *edge.twin()->cell();
			int prim_id_1 = get_cell_prim_id(&cell1);
			int prim_id_2 = get_cell_prim_id(&cell2);
			return prim_id_1==prim_id_2;
		}

		int get_cell_prim_id(const voronoi_diagram<coordinate_type>::cell_type* cell) {
			//get source index
			if (cell->source_category() == SOURCE_CATEGORY_SINGLE_POINT)
			{
				return -1;
				point_type pt = retrieve_point(*cell);
			}
			//segment index
			source_index_type seg_index = cell->source_index() - point_data_.size();
			int prim_index = seg_parent_prims_[seg_index];
			return prim_index;
		}

		int get_cell_segment_id(const voronoi_diagram<coordinate_type>::cell_type* cell) {
			//get source index
			if (cell->source_category()==SOURCE_CATEGORY_SINGLE_POINT)
			{
				return -1;
			}
			source_index_type index = cell->source_index() - point_data_.size();
		return index;
		}

		int get_cell_pt_id(const voronoi_diagram<coordinate_type>::cell_type* cell) {
			//get source index
			if (cell->source_category() == SOURCE_CATEGORY_SINGLE_POINT) return cell->source_index();
			return -1;
		}
		 
	segment_type retrieve_segment(const cell_type& cell) {
		source_index_type index = cell.source_index() - point_data_.size();
		return segment_data_[index];
	}

	point_type retrieve_point(const cell_type& cell) {
		source_index_type index = cell.source_index();
		source_category_type category = cell.source_category();
		if (category == boost::polygon::SOURCE_CATEGORY_SINGLE_POINT) {
			return point_data_[index];
		}
		index -= point_data_.size(); 
		if (category == boost::polygon::SOURCE_CATEGORY_SEGMENT_START_POINT) {
			return low(segment_data_[index]);
		}
		else {
			return high(segment_data_[index]);
		}
	}

	void clip_infinite_edge(
		const edge_type& edge, std::vector<point_type>* clipped_edge) {
		if (verbosity > 0)printf("clip_infinite_edge...\n");
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
		//add start point
		if (edge.vertex0() != NULL) {
			clipped_edge->push_back(
				point_type(edge.vertex0()->x(), edge.vertex0()->y()));
		}
		else {
			clipped_edge->push_back(
			point_type(edge.vertex1()->x(), edge.vertex1()->y()));

		}
		//add clipped point
		if (edge.vertex0() == NULL) {
			clipped_edge->push_back(point_type(
				origin.x() - direction.x() * koef,
				origin.y() - direction.y() * koef));
		}
				
		else {
			clipped_edge->push_back(point_type(
				origin.x() + direction.x() * koef,
				origin.y() + direction.y() * koef));
		}

	}

	void sample_curved_edge(const voronoi_diagram<coordinate_type>::edge_type& edge, std::vector<point_type>* sampled_edge) {
		if (verbosity > 0)printf("sample_curved_edge...\n");
		double max_dist = curve_sample_dist_ * scale_up_factor_;
		point_type point = edge.cell()->contains_point() ? retrieve_point(*edge.cell()) : retrieve_point(*edge.twin()->cell());
		segment_type segment = edge.cell()->contains_point() ?
			retrieve_segment(*edge.twin()->cell()) :
			retrieve_segment(*edge.cell());
		discretize(point, segment, max_dist, sampled_edge);
	}


	void set_near_point_values(const voronoi_diagram<coordinate_type>::edge_type* edge, std::vector<point_type>* samples, std::vector<point_type>* sampled_edge_sources) {
		const cell_type *cell = edge->cell();
				
		if (cell->contains_point()) { //for point sources
			point_type p0 = retrieve_point(*cell); 
			for (int i = 0; i < samples->size(); i++) { sampled_edge_sources->push_back(p0); }
		} 
		else {
			for (int i = 0; i < samples->size(); i++) {
				segment_type segment = retrieve_segment(*edge->cell());
				point_type &p_sample = samples->at(i);
				point_type p_proj = get_point_projection_point(p_sample, segment);
				sampled_edge_sources->push_back(p_proj);
			}
		}
		return;
	}

	void create_edge_hou_poly(GU_Detail *gdp, const voronoi_diagram<coordinate_type>::edge_type* edge,  std::vector<point_type>* samples) {
		std::vector<point_type> samples_sources; 
		GEO_PrimPoly *poly = (GEO_PrimPoly *)gdp->appendPrimitive(GA_PRIMPOLY);
		if (set_cell_prim_attr_) {
			const cell_type *cell = edge->cell();
			int cell_seg_id = -1;
			int cell_pt_id = -1;
			int cell_prim_id = -1;
			cell_prim_id = get_cell_prim_id(cell);
			cell_pt_id = get_cell_pt_id(cell);
			cell_seg_id = get_cell_segment_id(cell);

			GA_Offset offset = poly->getMapOffset();
			cell_prim_id_attrib_handle.set(offset, cell_prim_id);
			cell_seg_id_attrib_handle.set(offset, cell_seg_id);
			cell_pt_id_attrib_handle.set(offset, cell_pt_id);
		}
		if (set_near_point_attrib_) set_near_point_values(edge, samples, &samples_sources);

		for (int i = 0; i < samples->size(); ++i) {
			point_type p0 = samples->at(i);
			GA_Offset ptoff = addPointFromVDPt(gdp, p0);
			poly->appendVertex(ptoff);

			if (set_near_point_attrib_){
				point_type p_near = samples_sources.at(i);
				UT_Vector3 pos_near(p_near.x() / scale_up_factor_, 0.0, p_near.y()/ scale_up_factor_);
				pt_near_attrib_handle.set(ptoff, 0, p_near.x() / scale_up_factor_);
				pt_near_attrib_handle.set(ptoff, 2, p_near.y() / scale_up_factor_);
			}
		}
		return;
	}

	int create_hou_brect_poly(GU_Detail *gdp) {
		GEO_PrimPoly *poly = (GEO_PrimPoly *)gdp->appendPrimitive(GA_PRIMPOLY);
		//TODO add vertex attributes
		point_type pt = point_type(xl(brect_), yl(brect_));
		poly->appendVertex(addPointFromVDPt(gdp, point_type(xl(brect_), yl(brect_))));
		poly->appendVertex(addPointFromVDPt(gdp, point_type(xl(brect_), yh(brect_))));
		poly->appendVertex(addPointFromVDPt(gdp, point_type(xh(brect_), yh(brect_))));
		poly->appendVertex(addPointFromVDPt(gdp, point_type(xh(brect_), yl(brect_))));
		poly->appendVertex(addPointFromVDPt(gdp, point_type(xl(brect_), yl(brect_))));
		return 1;
	}

	float set_border_pos(point_type *p, point_type *p_prev, bool reverse= false) {
		point_type *p0 = reverse ? p_prev:p;
		point_type *p1 = reverse ? p : p_prev;
				
		point_type dir = point_type(p0->x()- p1->x() , double(p0->y() - p1->y() ));
		double dir_len = std::sqrt(dir.x()*dir.x() + dir.y()*dir.y()); //norm
		dir.x(dir.x() / dir_len);
		dir.y(dir.y()/ dir_len);

		double x_dist, y_dist, x_intersect_mag, y_intersect_mag;
		point_type pt_intersection;
		if (dir.x() == 0 || dir.y() == 0) {
			if(dir.x()==0){ 
				double y_side = dir.y() > 0 ? yh(brect_) : yl(brect_) ;
				pt_intersection = point_type(p1->x(), y_side ) ;
			}
			else{
				double x_side = dir.x() > 0 ? xh(brect_): xl(brect_) ;
				pt_intersection = point_type(x_side, p1->y());
			}
			return 0; //TODO, set intersection point
		}
		else {
			if (dir.x() > 0) { x_intersect_mag = (xh(brect_) - p1->x()) / dir.x(); }
			else { x_intersect_mag = (xl(brect_) - p1->x()) / dir.x(); }

			if (dir.y() > 0) { y_intersect_mag =(yh(brect_) - p1->y()) / dir.y(); }
			else { y_intersect_mag = (yl(brect_) - p1->y()) / dir.y(); }
		}
		x_intersect_mag = abs(x_intersect_mag);
		y_intersect_mag = abs(y_intersect_mag);

		double intersection_mag = 1.0* abs(x_intersect_mag)<abs(y_intersect_mag) ? x_intersect_mag: y_intersect_mag;
		if (mag_ == 0.0)return 0.0;
		p0->x(p1->x() + dir.x() * abs(intersection_mag) );
		p0->y(p1->y() + dir.y() * abs(intersection_mag) );

		return 1.0; //
	}

	void cull_outsides_points(std::vector<point_type>& sampled_edge)
	{
		int start_index = 0;
		point_type *p_prev = &sampled_edge.at(0); //change to pointer to redirect over iteration
		bool is_outside_last = is_point_outside_brect((*p_prev));// is_point_outside_brect(p_prev);

		for (int i = 1; i< sampled_edge.size(); i++)
		{
			//TODO clip infity lines against brect
			//TODO clip curves against brect
			point_type *p = &sampled_edge.at(i);
			bool is_outside = is_point_outside_brect((*p));

			if (is_outside && !is_outside_last ) {
				set_border_pos(p, p_prev);
			}
			else if (!is_outside && is_outside_last) {
				set_border_pos(p, p_prev, true);
			}
			else if (is_outside && is_outside_last) {
				sampled_edge.erase(sampled_edge.begin(), sampled_edge.begin() + i);
				return;
			}
			is_outside_last = is_outside;
			p_prev = p;
		}
	}

		// Traversing Voronoi edges using cell iterator.
	int iterate_primary_edges(GU_Detail *gdp, const voronoi_diagram<coordinate_type> &vd, std::vector<point_type>& points, std::vector<segment_type>& segments) {
		if (verbosity > 0)printf("iterate_primary_edges...\n");
		int result = 0;
		for (voronoi_diagram<coordinate_type>::const_cell_iterator it = vd.cells().begin();
			it != vd.cells().end(); ++it) {
			const voronoi_diagram<coordinate_type>::cell_type& cell = *it;
			const voronoi_diagram<coordinate_type>::edge_type* edge = cell.incident_edge();
			if (edge == NULL) {
				gdp->addWarning(GU_ERROR_MESSAGE, "unexpected graph, skipping NULL edge on output\n");
				//return -1;
			}
			else {
				// This is convenient way to iterate edges around Voronoi cell.
				do {
					if (!primary_edges_only_ || edge->is_primary()) {
						//if (remove_point_segments_ && (cell.source_category()== SOURCE_CATEGORY_SEGMENT_START_POINT || cell.source_category() == SOURCE_CATEGORY_SEGMENT_END_POINT) ) {}
						if (remove_point_segments_ && edge_is_internal_to_primid(*edge)) {}
						else {
							std::vector<point_type> samples;
							if (!edge->is_finite()) {
								clip_infinite_edge(*edge, &samples);
							}
							else {
								point_type vertex0(edge->vertex0()->x(), edge->vertex0()->y());
								samples.push_back(vertex0);
								point_type vertex1(edge->vertex1()->x(), edge->vertex1()->y());
								samples.push_back(vertex1);
								if (edge->is_curved() && do_curve_resample_) {
									sample_curved_edge(*edge, &samples);
								}

							}
							if (samples.size() > 1) {
								if (!keep_outside_) cull_outsides_points(samples);

								create_edge_hou_poly(gdp, edge, &samples);
							}
							++result;
						}
					}
					edge = edge->next();
				} while (edge != cell.incident_edge());
			}
		}
		return result;
	}



	int addVoronoiDiagramToHouMesh(GU_Detail *gdp, voronoi_diagram<coordinate_type>& vd, std::vector<point_type>& points, std::vector<segment_type>& segments) {
		//TODO add voronoi cells as prims and points
		if (verbosity > 0)printf("addVoronoiDiagramToHouMesh...\n");
		//create geometry for voronoi graph
		if (set_cell_prim_attr_) { 
			cell_seg_id_attrib_handle = gdp->addIntTuple(GA_ATTRIB_PRIMITIVE, cell_seg_attrib_name_ ,1); 
			cell_pt_id_attrib_handle = gdp->addIntTuple(GA_ATTRIB_PRIMITIVE, cell_pt_attrib_name_, 1);
			cell_prim_id_attrib_handle = gdp->addIntTuple(GA_ATTRIB_PRIMITIVE, cell_prim_attrib_name_, 1);
		}
		if(set_near_point_attrib_)pt_near_attrib_handle = gdp->addFloatTuple(GA_ATTRIB_POINT, pt_near_attrib_name_, 3);
		iterate_primary_edges(gdp, vd, points, segments);

		if (add_brect_poly_) { create_hou_brect_poly(gdp); }
		return 0;

	}
	void clear() {
		brect_initialized_ = false;
		point_data_.clear();
		segment_data_.clear();
		vd_.clear();
	}

	void update_brect(const point_type& point) {
		if (verbosity > 0)printf("update_brect...\n");
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
		bloat(brect_, side* bloat_);
	}
	public:

	void addVoronoiGraphToHoudiniGeo(GU_Detail *gdp, double test_parm, double mag, bool do_boundary_geo, bool do_resample, bool keep_outside, bool remove_point_segments) {
		do_curve_resample_ = do_resample;
		add_brect_poly_ = do_boundary_geo;
		keep_outside_ = keep_outside;
		remove_point_segments_ = remove_point_segments;
		
		
		bloat_ = test_parm;
		mag_ = mag;
		clear();
		compute_size_factor(gdp);
		houMeshLoader(gdp);
		gdp->clear();
		construct_brect();
		if (verbosity > 0) printf("addVoronoiGraphToHoudiniGeo...\n");
		// Construction of the Voronoi Diagram.
		voronoi_diagram<coordinate_type> vd;
		construct_voronoi(point_data_.begin(), point_data_.end(),
			segment_data_.begin(), segment_data_.end(),
			&vd);
		addVoronoiDiagramToHouMesh(gdp, vd, point_data_, segment_data_);
	}
	//class data
	//TODO fix chrash when creating a looped polygon on input
	//TODO add boundary polygon to input and leave option to remove both sides of its skeleton on output

	private:
	point_type shift_ = point_type(0.0, 0.0);
	rect_type brect_;
	std::vector<point_type> point_data_; 
	std::vector<segment_type> segment_data_;
	VB vb_;
	VD vd_;
	bool brect_initialized_;
	bool primary_edges_only_;
	bool internal_edges_only_;
	int verbosity = 0;
	int set_cell_prim_attr_ = true;

	int temp_ptnum = 0;
	bool do_cull_outside_ = true; //TODO remove this, probably not implemented
	bool keep_outside_ = true;
	bool remove_point_segments_ = true;
	bool add_brect_poly_ = true;
	bool do_curve_resample_ = true;
	bool set_near_point_attrib_ = true;
	double bloat_ = .55;
	double mag_ = 1.0;
	double max_coord_size_ = 65536; //1024*64 -arbitrary number
	double scale_up_factor_ = 1.0;  // set this by bounding box
	double curve_sample_dist_ = .05;
	std::vector<int> seg_parent_prims_; //list to map between line segments in vor geo and Houdini primitives
	GA_RWHandleI cell_seg_id_attrib_handle;
	GA_RWHandleI cell_pt_id_attrib_handle;
	GA_RWHandleI cell_prim_id_attrib_handle;
	GA_RWHandleF pt_near_attrib_handle;
	std::string cell_seg_attrib_name_ = "vor_cell_seg_id";
	std::string cell_pt_attrib_name_ = "vor_cell_pt_id";
	std::string cell_prim_attrib_name_ = "vor_cell_prim_id";
	std::string pt_near_attrib_name_ = "vor_source_pos";

};
