import math
from collections import defaultdict

from SingleUESTA.network import Network


SECONDS_PER_MINUTE = 60
FEET_PER_MILE = 5280
DELTA_T_H = 1.5  # seconds
DELTA_T_C = 1.0  # seconds
VEHICLE_LENGTH_FT = 15.0
ETA_C_DEFAULT = 0.1
FRANK_WOLFE_PRECISION = 1e-6
NO_PATH = "N/A"


class MultiClassSTA:
    """
    Multi-class (HDV/CV) static traffic assignment supporting UE and SO formulations.

    Parameters follow Team 2's CV scenario specification:
        - Two classes: human-driven vehicles (HDV) and connected vehicles (CV)
        - CV capacity derived from HDV capacity via Levin & Boyles (2016)
        - RSU indicators reduce CV costs via eta_c term when present
    """

    def __init__(self, network: Network, eta_c: float = ETA_C_DEFAULT):
        self.network = network
        self.classes = ("HDV", "CV")
        self.eta_c = eta_c
        self.rsu_links = set()
        self._precompute_link_data()
        self.configure_demand_split(1.0)

    # ------------------------------------------------------------------
    # Scenario configuration
    # ------------------------------------------------------------------
    def configure_demand_split(self, hdv_share: float):
        """Set HDV/CV demand split (CV share = 1 - HDV share)."""
        hdv_share = max(0.0, min(1.0, hdv_share))
        self.demand_share = {
            "HDV": hdv_share,
            "CV": max(0.0, 1.0 - hdv_share)
        }
        self.demands = self._split_demands()

    def configure_eta(self, eta_c: float):
        """Adjust the CV cost reduction parameter."""
        self.eta_c = max(0.0, eta_c)

    def configure_rsu_links(self, link_ids):
        """Specify which links contain RSUs (delta_a,RSU = 1)."""
        self.rsu_links = set(link_ids or [])

    # ------------------------------------------------------------------
    # Public solvers
    # ------------------------------------------------------------------
    def solve_user_equilibrium(self, maxIterations=200, targetGap=1e-4):
        """Frank-Wolfe for multiclass UE."""
        flows = self._all_or_nothing(self._compute_class_costs(self._zero_flows()))
        history = []

        for iteration in range(1, maxIterations + 1):
            costs = self._compute_class_costs(flows)
            search_direction = self._all_or_nothing(costs)
            step = self._frank_wolfe_step(flows, search_direction, use_marginal=False)
            flows = self._move_flows(flows, search_direction, step)
            costs = self._compute_class_costs(flows)
            shortest = self._all_or_nothing(costs)
            gap = self._relative_gap(costs, flows, shortest)
            history.append({"iteration": iteration, "gap": gap})
            if gap < targetGap:
                break

        results = self._build_results(flows, costs, history, objective="UE")
        return results

    def solve_system_optimal(self, maxIterations=200, targetGap=1e-4):
        """Frank-Wolfe for multiclass SO using marginal costs."""
        flows = self._all_or_nothing(self._compute_class_costs(self._zero_flows()))
        history = []

        for iteration in range(1, maxIterations + 1):
            marginal_costs = self._compute_marginal_costs(flows)
            search_direction = self._all_or_nothing(marginal_costs)
            step = self._frank_wolfe_step(flows, search_direction, use_marginal=True)
            flows = self._move_flows(flows, search_direction, step)
            costs = self._compute_class_costs(flows)
            marginal_costs = self._compute_marginal_costs(flows)
            shortest = self._all_or_nothing(marginal_costs)
            gap = self._relative_gap(marginal_costs, flows, shortest)
            history.append({"iteration": iteration, "gap": gap})
            if gap < targetGap:
                break

        results = self._build_results(flows, costs, history, objective="SO")
        return results

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------
    def _precompute_link_data(self):
        self.link_ids = list(self.network.link.keys())
        self.free_flow_time = {}
        self.length = {}
        self.capacity_hdv = {}
        self.capacity_cv = {}
        for ij, link in self.network.link.items():
            self.free_flow_time[ij] = link.freeFlowTime
            self.length[ij] = link.length
            self.capacity_hdv[ij] = link.capacity
            self.capacity_cv[ij] = self._calculate_cv_capacity(link)

    def _calculate_cv_capacity(self, link):
        length_ft = link.length * FEET_PER_MILE
        time_sec = link.freeFlowTime * SECONDS_PER_MINUTE
        if time_sec <= 0:
            return link.capacity
        speed = length_ft / time_sec
        numerator = speed * DELTA_T_H + VEHICLE_LENGTH_FT
        denominator = speed * DELTA_T_C + VEHICLE_LENGTH_FT
        if denominator <= 0:
            return link.capacity
        return link.capacity * (numerator / denominator)

    def _split_demands(self):
        demands = {cls: defaultdict(float) for cls in self.classes}
        for od_id, od in self.network.ODpair.items():
            for cls in self.classes:
                share = self.demand_share.get(cls, 0.0)
                demands[cls][od_id] = od.demand * share
        return demands

    def _zero_flows(self):
        flows = {cls: {ij: 0.0 for ij in self.link_ids} for cls in self.classes}
        return flows

    def _compute_class_costs(self, flows):
        costs = {cls: {} for cls in self.classes}
        for ij in self.link_ids:
            flow_h = flows["HDV"][ij]
            flow_c = flows["CV"][ij]
            cost_h, cost_c = self._link_costs(ij, flow_h, flow_c)
            costs["HDV"][ij] = cost_h
            costs["CV"][ij] = cost_c
        return costs

    def _compute_marginal_costs(self, flows):
        marginal_costs = {cls: {} for cls in self.classes}
        for ij in self.link_ids:
            flow_h = flows["HDV"][ij]
            flow_c = flows["CV"][ij]
            grad_h, grad_c = self._link_marginal_costs(ij, flow_h, flow_c)
            marginal_costs["HDV"][ij] = grad_h
            marginal_costs["CV"][ij] = grad_c
        return marginal_costs

    def _link_costs(self, link_id, flow_h, flow_c):
        t0 = self.free_flow_time[link_id]
        q_h = self.capacity_hdv[link_id]
        q_c = self.capacity_cv[link_id]
        if q_h <= 0 or q_c <= 0:
            return (t0, t0)

        ratio = flow_h / q_h + flow_c / q_c
        ratio = max(ratio, 0.0)
        congestion = pow(ratio, 4)
        base = 1.0 + congestion

        delta = 1.0 if link_id in self.rsu_links else 0.0
        flow_term = flow_c / q_c if q_c > 0 else 0.0
        reduction_argument = max(flow_term, delta)
        multiplier = 1.0 - self.eta_c * reduction_argument
        multiplier = max(multiplier, 0.0)

        cost_h = t0 * base
        cost_c = t0 * base * multiplier
        return cost_h, cost_c

    def _link_marginal_costs(self, link_id, flow_h, flow_c):
        t0 = self.free_flow_time[link_id]
        q_h = self.capacity_hdv[link_id]
        q_c = self.capacity_cv[link_id]
        if q_h <= 0 or q_c <= 0:
            return (t0, t0)

        ratio = flow_h / q_h + flow_c / q_c
        ratio = max(ratio, 0.0)
        congestion = pow(ratio, 4)
        base = 1.0 + congestion

        delta = 1.0 if link_id in self.rsu_links else 0.0
        flow_term = flow_c / q_c if q_c > 0 else 0.0
        reduction_argument = max(flow_term, delta)
        multiplier = max(1.0 - self.eta_c * reduction_argument, 0.0)

        cost_h = t0 * base
        cost_c = t0 * base * multiplier

        if ratio <= 0:
            return cost_h, cost_c

        d_ratio_dx_h = 1.0 / q_h
        d_ratio_dx_c = 1.0 / q_c
        d_congestion = 4.0 * pow(ratio, 3)
        d_base_dx_h = d_congestion * d_ratio_dx_h
        d_base_dx_c = d_congestion * d_ratio_dx_c

        if reduction_argument > delta or delta == 0.0:
            # Active derivative term (no RSU dominance or ratio surpasses delta)
            d_reduction_arg_dx_c = d_ratio_dx_c
        else:
            d_reduction_arg_dx_c = 0.0
        d_multiplier_dx_c = -self.eta_c * d_reduction_arg_dx_c

        d_cH_dxH = t0 * d_base_dx_h
        d_cH_dxC = t0 * d_base_dx_c
        d_cC_dxH = t0 * d_base_dx_h * multiplier
        d_cC_dxC = t0 * (d_base_dx_c * multiplier + base * d_multiplier_dx_c)

        grad_h = cost_h + flow_h * d_cH_dxH + flow_c * d_cC_dxH
        grad_c = cost_c + flow_h * d_cH_dxC + flow_c * d_cC_dxC
        return grad_h, grad_c

    def _all_or_nothing(self, costs):
        flows = {cls: {ij: 0.0 for ij in self.link_ids} for cls in self.classes}
        for cls in self.classes:
            demands = self.demands[cls]
            if all(d <= 0 for d in demands.values()):
                continue
            for origin in range(1, self.network.numZones + 1):
                backlink = self._shortest_path_tree(origin, costs[cls])
                for od_id, od in self.network.ODpair.items():
                    if od.origin != origin:
                        continue
                    demand = demands[od_id]
                    if demand <= 0:
                        continue
                    node = od.destination
                    while node != origin and backlink[node] != NO_PATH:
                        link_id = backlink[node]
                        flows[cls][link_id] += demand
                        node = self.network.link[link_id].tail
        return flows

    def _shortest_path_tree(self, origin, link_costs):
        backlink = {i: NO_PATH for i in self.network.node}
        label = {i: math.inf for i in self.network.node}
        label[origin] = 0.0
        scan_list = [origin]

        while scan_list:
            node = scan_list.pop(0)
            for link_id in self.network.node[node].leavingLinks:
                head = self.network.link[link_id].head
                temp_cost = label[node] + link_costs[link_id]
                if temp_cost < label[head]:
                    label[head] = temp_cost
                    backlink[head] = link_id
                    if head >= self.network.firstThroughNode:
                        scan_list.append(head)
        return backlink

    def _frank_wolfe_step(self, flows, direction, use_marginal):
        current = flows
        target = direction
        low, high = 0.0, 1.0
        alpha = 1.0
        while (high - low) > FRANK_WOLFE_PRECISION:
            alpha = 0.5 * (low + high)
            mixed = self._mix_flows(current, target, alpha)
            if use_marginal:
                costs = self._compute_marginal_costs(mixed)
            else:
                costs = self._compute_class_costs(mixed)
            derivative = 0.0
            for cls in self.classes:
                for ij in self.link_ids:
                    derivative += (target[cls][ij] - current[cls][ij]) * costs[cls][ij]
            if derivative < 0:
                low = alpha
            else:
                high = alpha
        return alpha

    def _mix_flows(self, current, target, alpha):
        mixed = {cls: {} for cls in self.classes}
        for cls in self.classes:
            for ij in self.link_ids:
                mixed[cls][ij] = alpha * target[cls][ij] + (1 - alpha) * current[cls][ij]
        return mixed

    def _move_flows(self, current, target, alpha):
        updated = {cls: {} for cls in self.classes}
        for cls in self.classes:
            for ij in self.link_ids:
                updated[cls][ij] = current[cls][ij] + alpha * (target[cls][ij] - current[cls][ij])
        return updated

    def _relative_gap(self, costs, flows, shortest):
        tsp = 0.0
        tstt = 0.0
        for cls in self.classes:
            for ij in self.link_ids:
                tsp += costs[cls][ij] * shortest[cls][ij]
                tstt += costs[cls][ij] * flows[cls][ij]
        if tsp <= 0:
            return 0.0
        return max(tstt / tsp - 1.0, 0.0)

    def _build_results(self, flows, costs, history, objective="UE"):
        tstt = 0.0
        total_demand = 0.0
        for cls in self.classes:
            for ij in self.link_ids:
                tstt += costs[cls][ij] * flows[cls][ij]
            total_demand += sum(self.demands[cls].values())
        avg_time = tstt / max(total_demand, 1e-9)
        return {
            "flows": flows,
            "costs": costs,
            "history": history,
            "tstt": tstt,
            "avg_time": avg_time,
            "objective": objective
        }
