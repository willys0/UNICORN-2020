#ifndef SHAPE_EXTRACTION_H
#define SHAPE_EXTRACTION_H


class shape_extraction
{
public:
    shape_extraction();

	void adaptive_breaK_point();
	void static_map_filter();
	void polygon_extraction();
	void polygon_attribute_extraction();

private:
	void extract_corners(int startpoint,int endpoint, int length,int shape_nr);
	void search_longest(int startpoint, int current_point,int end_point, int length, float distance_S, int itteration, int max_itteration, int *best_point, float *best_dist);

};

#endif