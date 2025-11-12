#include <bits/stdc++.h>
using namespace std;

// Simple C++ implementation of the time-optimizing PSO from the Python prototype.
// Outputs per-UAV CSV files with columns t,x,y,z and a history CSV with gbest cost.

// NOTE: This is a straightforward, single-threaded implementation intended for
// correctness and portability, not raw performance. It avoids external deps.

struct Config {
    int n_uavs = 3;
    int T = 10;
    double dt_base = 1.0;
    double max_speed = 3.0;
    double min_sep = 2.0;
    array<pair<double,double>,3> space_bounds = { make_pair(0.0,40.0), make_pair(0.0,40.0), make_pair(0.5,10.0) };
    //vector<vector<double>> starts{{2,2,3},{2,8,3},{2,14,3}};
    //vector<vector<double>> goals{{35,35,4},{35,30,6},{35,25,3}};
    vector<vector<double>> starts{{0,0,5},{0,20,5},{20,0,5}};
    vector<vector<double>> goals{{20,20,5},{20,0,5},{0,20,5}};
};

// Note: random helpers are created per-run inside main using a single rng to
// ensure reproducibility and parity with the Python prototype (seeded RNG).

using Vec = vector<double>;

inline void clamp_pos(vector<vector<vector<double>>> &traj, const Config &cfg){
    int n = cfg.n_uavs;
    int T = cfg.T;
    for(int u=0; u<n; ++u) for(int t=0;t<T;++t) for(int d=0; d<3; ++d){
        double low = cfg.space_bounds[d].first;
        double high = cfg.space_bounds[d].second;
        traj[u][t][d] = min(max(traj[u][t][d], low), high);
    }
}

// cumulative times from deltas
vector<vector<double>> cumulative_times_from_deltas(const vector<vector<double>> &deltas){
    int n = deltas.size();
    int m = deltas[0].size();
    vector<vector<double>> times(n, vector<double>(m+1, 0.0));
    for(int i=0;i<n;++i){
        for(int k=0;k<m;++k) times[i][k+1] = times[i][k] + deltas[i][k];
    }
    return times;
}

// linear interpolation for 1D index
vector<double> interp_pos_at_time(const vector<vector<double>> &traj, const vector<double> &times, double t_query){
    int T = (int)traj.size();
    if(t_query <= times[0]) return traj[0];
    if(t_query >= times.back()) return traj.back();
    int idx = (int)(upper_bound(times.begin(), times.end(), t_query) - times.begin()) - 1;
    double t0 = times[idx]; double t1 = times[idx+1];
    const vector<double> &p0 = traj[idx];
    const vector<double> &p1 = traj[idx+1];
    double alpha = (t_query - t0) / (t1 - t0 + 1e-12);
    vector<double> out(3);
    for(int d=0; d<3; ++d) out[d] = (1-alpha)*p0[d] + alpha*p1[d];
    return out;
}

// cost terms

// Distancia mínima entre dos segmentos en 3D
double segment_min_dist(const vector<double> &p1, const vector<double> &p2, const vector<double> &q1, const vector<double> &q2) {
    // Algoritmo estándar: https://stackoverflow.com/a/14772574
    vector<double> u(3), v(3), w0(3);
    for(int d=0; d<3; ++d) {
        u[d] = p2[d] - p1[d];
        v[d] = q2[d] - q1[d];
        w0[d] = p1[d] - q1[d];
    }
    double a = u[0]*u[0] + u[1]*u[1] + u[2]*u[2];
    double b = u[0]*v[0] + u[1]*v[1] + u[2]*v[2];
    double c = v[0]*v[0] + v[1]*v[1] + v[2]*v[2];
    double d = u[0]*w0[0] + u[1]*w0[1] + u[2]*w0[2];
    double e = v[0]*w0[0] + v[1]*w0[1] + v[2]*w0[2];
    double D = a*c - b*b;
    double sc, tc;
    if(D < 1e-12) {
        sc = 0.0;
        tc = (c > 1e-12) ? (e/c) : 0.0;
    } else {
        sc = (b*e - c*d) / D;
        tc = (a*e - b*d) / D;
    }
    sc = std::min<double>(std::max<double>(sc, 0.0), 1.0);
    tc = std::min<double>(std::max<double>(tc, 0.0), 1.0);
    vector<double> pt1(3), pt2(3);
    for(int d=0; d<3; ++d) {
        pt1[d] = p1[d] + sc*u[d];
        pt2[d] = q1[d] + tc*v[d];
    }
    double dist = sqrt((pt1[0]-pt2[0])*(pt1[0]-pt2[0]) + (pt1[1]-pt2[1])*(pt1[1]-pt2[1]) + (pt1[2]-pt2[2])*(pt1[2]-pt2[2]));
    return dist;
}

double path_length(const vector<vector<vector<double>>> &trajectories){
    double L = 0.0;
    for(size_t u=0; u<trajectories.size(); ++u){
        for(size_t t=1; t<trajectories[u].size(); ++t){
            double s=0; for(int d=0; d<3; ++d){ double diff = trajectories[u][t][d] - trajectories[u][t-1][d]; s += diff*diff; }
            L += sqrt(s);
        }
    }
    return L;
}

double speed_penalty_timeopt(const vector<vector<vector<double>>> &trajectories, const vector<vector<double>> &deltas, const Config &cfg){
    int n = cfg.n_uavs; int T = cfg.T;
    double pen = 0.0;
    for(int u=0; u<n; ++u){
        for(int t=0; t<T-1; ++t){
            double s=0; for(int d=0; d<3; ++d){ double diff = trajectories[u][t+1][d]-trajectories[u][t][d]; s += diff*diff; }
            double dist = sqrt(s);
            double delta = max(deltas[u][t], 1e-6);
            double speed = dist / delta;
            double excess = max(0.0, speed - cfg.max_speed);
            pen += excess*excess * 100.0;
            if(delta < 0.05) pen += (0.05 - delta)*(0.05 - delta) * 1000.0;
        }
    }
    return pen;
}

double collision_penalty_timeopt(const vector<vector<vector<double>>> &trajectories, const vector<vector<double>> &deltas, const Config &cfg){
    // Penalización por colisión en puntos discretos (como antes)
    auto times = cumulative_times_from_deltas(deltas);
    int n = cfg.n_uavs; int T = cfg.T;
    double pen = 0.0;
    for(int u=0; u<n; ++u){
        for(int i=0; i<T; ++i){
            double t_u = times[u][i];
            const vector<double> &p_u = trajectories[u][i];
            for(int v=u+1; v<n; ++v){
                vector<double> p_v = interp_pos_at_time(trajectories[v], times[v], t_u);
                double s=0; for(int d=0; d<3; ++d){ double diff = p_u[d] - p_v[d]; s += diff*diff; }
                double dist = sqrt(s);
                if(dist < cfg.min_sep) pen += (cfg.min_sep - dist)*(cfg.min_sep - dist) * 2000.0;
            }
        }
    }
    // Penalización por colisión en los segmentos entre puntos
    for(int seg_idx=0; seg_idx<T-1; ++seg_idx){
        for(int i1=0; i1<n; ++i1){
            const vector<double> &p1 = trajectories[i1][seg_idx];
            const vector<double> &p2 = trajectories[i1][seg_idx+1];
            for(int i2=i1+1; i2<n; ++i2){
                const vector<double> &q1 = trajectories[i2][seg_idx];
                const vector<double> &q2 = trajectories[i2][seg_idx+1];
                double dseg = segment_min_dist(p1, p2, q1, q2);
                if(dseg < cfg.min_sep) pen += (cfg.min_sep - dseg)*(cfg.min_sep - dseg) * 2000.0;
            }
        }
    }
    return pen;
}


double boundary_penalty(const vector<vector<vector<double>>> &trajectories, const Config &cfg){
    double pen = 0.0;
    int n = cfg.n_uavs; int T = cfg.T;
    for(int u=0; u<n; ++u) for(int t=0;t<T;++t) for(int d=0; d<3; ++d){
        double low = cfg.space_bounds[d].first; double high = cfg.space_bounds[d].second;
        double v = trajectories[u][t][d];
        if(v < low) pen += (low - v)*(low - v) * 100.0;
        if(v > high) pen += (v - high)*(v - high) * 100.0;
    }
    return pen;
}


double objective_timeopt_flat(const Vec &flat, const Config &cfg){
    int pos_len = cfg.n_uavs * cfg.T * 3;
    int del_len = cfg.n_uavs * (cfg.T-1);
    // unpack
    vector<vector<vector<double>>> traj(cfg.n_uavs, vector<vector<double>>(cfg.T, vector<double>(3)));
    vector<vector<double>> deltas(cfg.n_uavs, vector<double>(cfg.T-1));
    int idx=0;
    for(int u=0; u<cfg.n_uavs; ++u) for(int t=0;t<cfg.T;++t) for(int d=0;d<3;++d) traj[u][t][d] = flat[idx++];
    for(int u=0; u<cfg.n_uavs; ++u) for(int t=0;t<cfg.T-1;++t) deltas[u][t] = flat[idx++];
    // penalties
    double start_err=0, goal_err=0;
    for(int u=0; u<cfg.n_uavs; ++u){ for(int d=0; d<3; ++d){ double v0 = traj[u][0][d] - cfg.starts[u][d]; start_err += v0*v0; double v1 = traj[u][cfg.T-1][d] - cfg.goals[u][d]; goal_err += v1*v1; } }
    start_err *= 1000.0; goal_err *= 1000.0;
    double L = path_length(traj);
    double sp = speed_penalty_timeopt(traj, deltas, cfg);
    double coll = collision_penalty_timeopt(traj, deltas, cfg);
    double bnd = boundary_penalty(traj, cfg);
    double pos_smooth = 0.0; // simple second-diff smoothness
    for(int u=0; u<cfg.n_uavs; ++u){
        for(int t=0; t<cfg.T-2; ++t){ double s=0; for(int d=0;d<3;++d){ double a=traj[u][t+2][d]-2*traj[u][t+1][d]+traj[u][t][d]; s+=a*a; } pos_smooth += sqrt(s); }
    }
    pos_smooth *= 500.0;
    // time smoothness approximate: sum abs diff of deltas
    double time_smooth = 0.0;
    for(int u=0; u<cfg.n_uavs; ++u) for(int t=0; t<cfg.T-2; ++t) time_smooth += fabs(deltas[u][t+1]-deltas[u][t]);
    time_smooth *= 50.0;
    double cost = L + pos_smooth + time_smooth + sp + coll + bnd + start_err + goal_err;
    return cost;
}

// Save CSVs
void save_trajectories_csv(const vector<vector<vector<double>>> &traj, const vector<vector<double>> &deltas, const string &prefix="uav_traj_cpp"){
    int n = traj.size(); int T = traj[0].size();
    auto times = cumulative_times_from_deltas(deltas);
    for(int u=0; u<n; ++u){
        string fname = string("output/") + prefix + "_uav" + to_string(u) + ".csv";
        ofstream f(fname);
        f << "t,x,y,z\n";
        for(int t=0;t<T;++t){
            double tt = times[u][t];
            f << tt << "," << traj[u][t][0] << "," << traj[u][t][1] << "," << traj[u][t][2] << "\n";
        }
        f.close();
    }
}

int main(int argc, char** argv){
    ios::sync_with_stdio(false);
    cin.tie(nullptr);
    Config cfg;
    // PSO params — align with Python prototype defaults
    int n_particles = 200;
    int iters = 500;
    double w = 0.6, c1 = 1.8, c2 = 1.8;
    // seed
    std::mt19937_64 rng(42);
    // distributions and convenience lambdas (use same rng for reproducibility)
    std::normal_distribution<double> norm01(0.0, 1.0);
    std::uniform_real_distribution<double> uni01(0.0, 1.0);
    auto randn = [&](double mean=0.0, double std=1.0){ return mean + std * norm01(rng); };
    auto randu = [&](){ return uni01(rng); };

    int pos_dim = cfg.n_uavs * cfg.T * 3;
    int del_dim = cfg.n_uavs * (cfg.T-1);
    int dim = pos_dim + del_dim;

    vector<Vec> particles(n_particles, Vec(dim, 0.0));
    vector<Vec> velocities(n_particles, Vec(dim, 0.0));
    vector<Vec> pbest(n_particles, Vec(dim, 0.0));
    vector<double> pbest_cost(n_particles, 1e300);
    Vec gbest(dim, 0.0); double gbest_cost = 1e300;

    // init
    for(int p=0;p<n_particles;++p){
        // straight-line traj + noise
        int idx=0;
        for(int u=0; u<cfg.n_uavs; ++u){
            for(int t=0;t<cfg.T;++t){
                for(int d=0; d<3; ++d){
                    double v = cfg.starts[u][d] + (cfg.goals[u][d]-cfg.starts[u][d]) * (double)t / (cfg.T-1);
                    v += randn(0.0, 0.6);
                    particles[p][idx++] = v;
                }
            }
        }
        for(int u=0; u<cfg.n_uavs; ++u) for(int t=0;t<cfg.T-1;++t) particles[p][idx++] = max(0.05, cfg.dt_base + randn(0.0, 0.1));
        for(int i=0;i<dim;++i) velocities[p][i] = randn(0.0, 0.5);
        double cost = objective_timeopt_flat(particles[p], cfg);
        pbest[p] = particles[p]; pbest_cost[p] = cost;
        if(cost < gbest_cost){ gbest_cost = cost; gbest = particles[p]; }
    }

    vector<double> history;
    history.reserve(iters);

    for(int it=0; it<iters; ++it){
        for(int p=0;p<n_particles;++p){
            for(int k=0;k<dim;++k){
                double r1 = randu(); double r2 = randu();
                velocities[p][k] = w * velocities[p][k] + c1*r1*(pbest[p][k] - particles[p][k]) + c2*r2*(gbest[k] - particles[p][k]);
                particles[p][k] += velocities[p][k];
            }
            // clamp positions and deltas
            int idx=0;
            for(int u=0; u<cfg.n_uavs; ++u){
                for(int t=0;t<cfg.T;++t){
                    for(int d=0; d<3; ++d){
                        double low = cfg.space_bounds[d].first, high = cfg.space_bounds[d].second;
                        particles[p][idx] = min(max(particles[p][idx], low), high);
                        ++idx;
                    }
                }
            }
            for(int u=0; u<cfg.n_uavs; ++u) for(int t=0;t<cfg.T-1;++t){ particles[p][idx] = min(max(particles[p][idx], 0.05), 10.0); ++idx; }

            double cost = objective_timeopt_flat(particles[p], cfg);
            if(cost < pbest_cost[p]){ pbest_cost[p] = cost; pbest[p] = particles[p]; }
            if(cost < gbest_cost){ gbest_cost = cost; gbest = particles[p]; }
        }
        history.push_back(gbest_cost);
        if((it+1) % 25 == 0 || it==0) cout << "Iter " << (it+1) << "/" << iters << ", best cost=" << gbest_cost << "\n";
    }

    // unpack gbest
    vector<vector<vector<double>>> best_traj(cfg.n_uavs, vector<vector<double>>(cfg.T, vector<double>(3)));
    vector<vector<double>> best_deltas(cfg.n_uavs, vector<double>(cfg.T-1));
    int idx=0;
    for(int u=0; u<cfg.n_uavs; ++u) for(int t=0;t<cfg.T;++t) for(int d=0; d<3; ++d) best_traj[u][t][d] = gbest[idx++];
    for(int u=0; u<cfg.n_uavs; ++u) for(int t=0;t<cfg.T-1;++t) best_deltas[u][t] = gbest[idx++];

    // ensure output dir
    system("mkdir -p output");
    save_trajectories_csv(best_traj, best_deltas, "uav_traj_cpp");
    // save history
    ofstream fh("output/history_cpp.csv"); fh << "iter,cost\n"; for(size_t i=0;i<history.size();++i) fh << i << "," << history[i] << "\n"; fh.close();
    cout << "Saved CSVs in output/" << endl;
    return 0;
}
