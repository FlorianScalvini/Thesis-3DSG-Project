#ifndef OUTDOORNAV_TRACKINGALGORITHM_H
#define OUTDOORNAV_TRACKINGALGORITHM_H
#include <vector>
#include <memory>
#include "ObjectTracking.h"
#include "lapjv.h"

class TrackingAlgorithm {
public:
    virtual std::vector<std::shared_ptr<ObjectTracking>> update(const std::vector<ObjectBoundingBox>& objects) = 0;
    static std::vector<std::shared_ptr<ObjectTracking>> jointStracks(const std::vector<std::shared_ptr<ObjectTracking>> &a_tlist,
                                                                  const std::vector<std::shared_ptr<ObjectTracking>> &b_tlist) ;

    static std::vector<std::shared_ptr<ObjectTracking>> subStracks(const std::vector<std::shared_ptr<ObjectTracking>> &a_tlist,
                                                                const std::vector<std::shared_ptr<ObjectTracking>> &b_tlist);

    static void removeDuplicateStracks(const std::vector<std::shared_ptr<ObjectTracking>> &a_stracks, const std::vector<std::shared_ptr<ObjectTracking>> &b_stracks,
                                       std::vector<std::shared_ptr<ObjectTracking>> &a_res, std::vector<std::shared_ptr<ObjectTracking>> &b_res) ;

    static std::vector<std::vector<float>> calcIouDistance(const std::vector<std::shared_ptr<ObjectTracking>> &a_tracks,
                                                           const std::vector<std::shared_ptr<ObjectTracking>> &b_tracks);

    static std::vector<std::vector<float>> calcIous(const std::vector<Rect> &a_rect, const std::vector<Rect> &b_rect);


    static void linearAssignment(const std::vector<std::vector<float>> &cost_matrix, const int &cost_matrix_size,
                                 const int &cost_matrix_size_size, const float &thresh, std::vector<std::vector<int>> &matches,
                                 std::vector<int> &a_unmatched, std::vector<int> &b_unmatched);

    static double execLapjv(const std::vector<std::vector<float>> &cost, std::vector<int> &rowsol, std::vector<int> &colsol,
                            bool extend_cost, float cost_limit, bool return_cost=true);

};


#endif //OUTDOORNAV_TRACKINGALGORITHM_H
