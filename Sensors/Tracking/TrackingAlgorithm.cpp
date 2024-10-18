#include "TrackingAlgorithm.h"


std::vector<std::shared_ptr<ObjectTracking>> TrackingAlgorithm::jointStracks(const std::vector<std::shared_ptr<ObjectTracking>> &a_tlist,
                                                                             const std::vector<std::shared_ptr<ObjectTracking>> &b_tlist) {
    std::map<int, int> exists;
    std::vector<std::shared_ptr<ObjectTracking>> res;
    for (const auto & i : a_tlist)
    {
        exists.emplace(i->getTrackId(), 1);
        res.push_back(i);
    }
    for (const auto & i : b_tlist)
    {
        const int &tid = (int) i->getTrackId();
        if (!exists[tid] || exists.count(tid) == 0)
        {
            exists[tid] = 1;
            res.push_back(i);
        }
    }
    return res;
}

std::vector<std::shared_ptr<ObjectTracking>> TrackingAlgorithm::subStracks(const std::vector<std::shared_ptr<ObjectTracking>> &a_tlist,
                                                                           const std::vector<std::shared_ptr<ObjectTracking>> &b_tlist) {
    std::map<int, std::shared_ptr<ObjectTracking>> stracks;
    for (const auto & i : a_tlist)
    {
        stracks.emplace(i->getTrackId(), i);
    }

    for (const auto & i : b_tlist)
    {
        const int &tid = (int) i->getTrackId();
        if (stracks.count(tid) != 0)
        {
            stracks.erase(tid);
        }
    }

    std::vector<std::shared_ptr<ObjectTracking>> res;
    std::map<int, std::shared_ptr<ObjectTracking>>::iterator it;
    for (it = stracks.begin(); it != stracks.end(); ++it)
    {
        res.push_back(it->second);
    }

    return res;
}

void TrackingAlgorithm::removeDuplicateStracks(const std::vector<std::shared_ptr<ObjectTracking>> &a_stracks,
                                               const std::vector<std::shared_ptr<ObjectTracking>> &b_stracks,
                                               std::vector<std::shared_ptr<ObjectTracking>> &a_res,
                                               std::vector<std::shared_ptr<ObjectTracking>> &b_res) {

    const auto ious = calcIouDistance(a_stracks, b_stracks);

    std::vector<std::pair<size_t, size_t>> overlapping_combinations;
    for (size_t i = 0; i < ious.size(); i++)
    {
        for (size_t j = 0; j < ious[i].size(); j++)
        {
            if (ious[i][j] < 0.15)
            {
                overlapping_combinations.emplace_back(i, j);
            }
        }
    }

    std::vector<bool> a_overlapping(a_stracks.size(), false), b_overlapping(b_stracks.size(), false);
    for (const auto &[a_idx, b_idx] : overlapping_combinations)
    {
        const unsigned int timep = a_stracks[a_idx]->getFrameId() - a_stracks[a_idx]->getStartFrameId();
        const unsigned int timeq = b_stracks[b_idx]->getFrameId() - b_stracks[b_idx]->getStartFrameId();
        if (timep > timeq)
        {
            b_overlapping[b_idx] = true;
        }
        else
        {
            a_overlapping[a_idx] = true;
        }
    }

    for (size_t ai = 0; ai < a_stracks.size(); ai++)
    {
        if (!a_overlapping[ai])
        {
            a_res.push_back(a_stracks[ai]);
        }
    }

    for (size_t bi = 0; bi < b_stracks.size(); bi++)
    {
        if (!b_overlapping[bi])
        {
            b_res.push_back(b_stracks[bi]);
        }
    }
}


std::vector<std::vector<float> > TrackingAlgorithm::calcIouDistance(const std::vector<std::shared_ptr<ObjectTracking>> &a_tracks,
                                                            const std::vector<std::shared_ptr<ObjectTracking>> &b_tracks)
{
    std::vector<Rect> a_rects, b_rects;
    for (const auto & a_track : a_tracks)
    {
        a_rects.push_back(a_track->getRect());
    }

    for (const auto & b_track : b_tracks)
    {
        b_rects.push_back(b_track->getRect());
    }

    const auto ious = calcIous(a_rects, b_rects);

    std::vector<std::vector<float>> cost_matrix;
    for (const auto & i : ious)
    {
        std::vector<float> iou;
        for (float j : i)
        {
            iou.push_back(1 - j);
        }
        cost_matrix.push_back(iou);
    }

    return cost_matrix;
}


std::vector<std::vector<float>> TrackingAlgorithm::calcIous(const std::vector<Rect> &a_rect, const std::vector<Rect> &b_rect){
    std::vector<std::vector<float>> ious;
    if (a_rect.size() * b_rect.size() == 0)
        return ious;
    ious.resize(a_rect.size());
    for (auto & iou : ious)
        iou.resize(b_rect.size());
    for (size_t bi = 0; bi < b_rect.size(); bi++)
        for (size_t ai = 0; ai < a_rect.size(); ai++)
            ious[ai][bi] = 1 - Rect::calcIoU(a_rect[ai],b_rect[bi]);
    return ious;
}

void
TrackingAlgorithm::linearAssignment(const std::vector<std::vector<float>> &cost_matrix, const int &cost_matrix_size,
                                    const int &cost_matrix_size_size, const float &thresh,
                                    std::vector<std::vector<int>> &matches, std::vector<int> &a_unmatched,
                                    std::vector<int> &b_unmatched){

    if (cost_matrix.empty())
    {
        for (int i = 0; i < cost_matrix_size; i++)
        {
            a_unmatched.push_back(i);
        }
        for (int i = 0; i < cost_matrix_size_size; i++)
        {
            b_unmatched.push_back(i);
        }
        return;
    }

    std::vector<int> rowsol; std::vector<int> colsol;
    execLapjv(cost_matrix, rowsol, colsol, true, thresh);
    for (size_t i = 0; i < rowsol.size(); i++)
    {
        if (rowsol[i] >= 0)
        {
            std::vector<int> match;
            match.push_back(i);
            match.push_back(rowsol[i]);
            matches.push_back(match);
        }
        else
        {
            a_unmatched.push_back(i);
        }
    }

    for (size_t i = 0; i < colsol.size(); i++)
    {
        if (colsol[i] < 0)
        {
            b_unmatched.push_back(i);
        }
    }

}



double TrackingAlgorithm::execLapjv(const std::vector<std::vector<float>> &cost, std::vector<int> &rowsol,
                                    std::vector<int> &colsol, bool extend_cost, float cost_limit, bool return_cost)
{
    std::vector<std::vector<float> > cost_c;
    cost_c.assign(cost.begin(), cost.end());

    std::vector<std::vector<float> > cost_c_extended;

    int n_rows = (int)cost.size();
    int n_cols = (int)cost[0].size();
    rowsol.resize(n_rows);
    colsol.resize(n_cols);

    int n = 0;
    if (n_rows == n_cols)
    {
        n = n_rows;
    }
    else
    {
        if (!extend_cost)
        {
            throw std::runtime_error("The `extend_cost` variable should set True");
        }
    }

    if (extend_cost || cost_limit < std::numeric_limits<float>::max())
    {
        n = n_rows + n_cols;
        cost_c_extended.resize(n);
        for (auto & i : cost_c_extended)
            i.resize(n);

        if (cost_limit < std::numeric_limits<float>::max())
        {
            for (auto & i : cost_c_extended)
            {
                for (float & j : i)
                {
                    j = (float)(cost_limit / 2.0);
                }
            }
        }
        else
        {
            float cost_max = -1;
            for (auto & i : cost_c)
            {
                for (float j : i)
                {
                    if (j > cost_max)
                        cost_max = j;
                }
            }
            for (auto & i : cost_c_extended)
            {
                for (float & j : i)
                {
                    j = cost_max + 1;
                }
            }
        }

        for (size_t i = n_rows; i < cost_c_extended.size(); i++)
        {
            for (size_t j = n_cols; j < cost_c_extended[i].size(); j++)
            {
                cost_c_extended[i][j] = 0;
            }
        }
        for (int i = 0; i < n_rows; i++)
        {
            for (int j = 0; j < n_cols; j++)
            {
                cost_c_extended[i][j] = cost_c[i][j];
            }
        }

        cost_c.clear();
        cost_c.assign(cost_c_extended.begin(), cost_c_extended.end());
    }

    double **cost_ptr;
    cost_ptr = new double *[sizeof(double *) * n];
    for (int i = 0; i < n; i++)
        cost_ptr[i] = new double[sizeof(double) * n];

    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            cost_ptr[i][j] = cost_c[i][j];
        }
    }

    int* x_c = new int[sizeof(int) * n];
    int *y_c = new int[sizeof(int) * n];

    int ret = lapjv_internal(n, cost_ptr, x_c, y_c);
    if (ret != 0)
    {
        throw std::runtime_error("The result of lapjv_internal() is invalid.");
    }

    double opt = 0.0;

    if (n != n_rows)
    {
        for (int i = 0; i < n; i++)
        {
            if (x_c[i] >= n_cols)
                x_c[i] = -1;
            if (y_c[i] >= n_rows)
                y_c[i] = -1;
        }
        for (int i = 0; i < n_rows; i++)
        {
            rowsol[i] = x_c[i];
        }
        for (int i = 0; i < n_cols; i++)
        {
            colsol[i] = y_c[i];
        }

        if (return_cost)
        {
            for (size_t i = 0; i < rowsol.size(); i++)
            {
                if (rowsol[i] != -1)
                {
                    opt += cost_ptr[i][rowsol[i]];
                }
            }
        }
    }
    else if (return_cost)
    {
        for (size_t i = 0; i < rowsol.size(); i++)
        {
            opt += cost_ptr[i][rowsol[i]];
        }
    }

    for (int i = 0; i < n; i++)
    {
        delete[]cost_ptr[i];
    }
    delete[]cost_ptr;
    delete[]x_c;
    delete[]y_c;

    return opt;
}
