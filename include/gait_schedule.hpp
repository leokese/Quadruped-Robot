#pragma once

#include "common/types.hpp"

struct Gait
{
    int steps;
    int n_qs;
    int n_qs_init; // 初始全接触支持阶段的数量
    int n_qs_end; // 末尾全接触支持阶段的数量
    int n_ds;
    int nsteps;
    
    Gait(int steps_, int n_qs_, int n_ds_)
        : steps(steps_), n_qs(n_qs_), n_ds(n_ds_)
        {
            nsteps = steps * (2 * n_qs + 2 * n_ds);
            n_qs_init = (n_qs + 1) / 2; // 初始全接触支持阶段的数量（向上取整）
            n_qs_end = n_qs - n_qs_init; // 末尾全接触支持阶段的数量
        }

    std::vector<std::vector<bool>> generateFootStates()
    {
        std::vector<std::vector<bool>> contact_states;

        for (int i = 0; i < steps; ++i)
        {
            // 第零阶段：全接触支持（开始为3个时刻接触，结尾为2个时刻接触）
            for (int j = 0; j < n_qs_init; ++j)
            {
                contact_states.push_back({true, true, true, true});
            }

            // 第一阶段：第一组双足接触（例如 swing 第 0 和 3 号足）
            for (int j = 0; j < n_ds; ++j)
            {
                contact_states.push_back({false, true, true, false});       
            }

            // 第二阶段：全接触支持
            for (int j = 0; j < n_qs; ++j)
            {
                contact_states.push_back({true, true, true, true});
            }
            
            // 第三阶段：第二组双足接触（例如 swing 第 1 和 2 号足）
            for (int j = 0; j < n_ds; ++j)
            {
                contact_states.push_back({true, false, false, true});
            }

            // 第四阶段：全接触支持
            for (int j = 0; j < n_qs_end; ++j)
            {
                contact_states.push_back({true, true, true, true});
            }
            
        }
        
        return contact_states;

    }

};