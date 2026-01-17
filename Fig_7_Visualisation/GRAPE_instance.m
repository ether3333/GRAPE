%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GRAPE Algorithm for Multi-Robot Task Allocation
% By Inmo Jang, 06.Oct.2015
% Modified, 12.Jan.2015
% Modified, 15.Jul.2015
% Modified, 23.Jun.2017
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialisation (1) - Generation of Random Scenario
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Tasks/Agents Location
t_location_limit = [9500 9500]; % Task locations range: max(X) max(Y) (metre)
a_location_limit = [9300 9300]; % Agent locations range: max(X) & max(Y) (metre)

% Task Demand or Reward
t_demand_mean = 1000*n/m; % 15.(Jul.2016) In order to maintain the level of individual utilities regardless of #Tasks or #Agents

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
t_location = zeros(m,2);    % task location - 2D
a_location = zeros(n,2);    % agent location - 2D
t_demand = zeros(m,1);      % task rewards

%% MST 수정 1 : Agent Communication Range 조정
Comm_distance = 9300;         % Communication range of each robot
Gap_agent = 15;             % Minimum spatial distance between any two robots
Gap_task = 200;             % Minimum spatial distance between any two tasks

% Generation of Task information
for t=1:m
    ok = 0;
    while ok == 0
        t_location(t,:) = [random('Uniform',-t_location_limit(1),t_location_limit(1)) ...
            random('Uniform',-t_location_limit(2),t_location_limit(2))];
        if t_location(t,1) > - a_location_limit(1) - 100 && t_location(t,1) < a_location_limit(1) + 100 ...
                && t_location(t,2) > - a_location_limit(2) -100 && t_location(t,2) < a_location_limit(2) + 100
            ok = 0;
        else
            ok = 1;
            for k=1:t-1
                if norm(t_location(k,:) - t_location(t,:)) < Gap_task
                    ok = 0;
                end
            end
        end
    end
    t_demand(t) = abs(random('Uniform',t_demand_mean*1,t_demand_mean*2));
end

%% MST 수정 2 : Agent 분포 방식 변경（여기서는 손댄 것 없음 － Main_visual.m에서 설정）
% Generation of Agent information
for i=1:n
    ok = 0;
    while ok == 0
        a_location(i,:) = [random('Uniform',-a_location_limit(1), a_location_limit(1)) random('Uniform',-a_location_limit(2), a_location_limit(2))];
        switch Deployment
            % Randomely distribute agents as a circle
            case 1
                if norm(a_location(i,:)) < a_location_limit(1) % Case (1) Circle
                    ok = 1;
                    for k=1:i-1
                        if norm(a_location(k,:) - a_location(i,:)) < Gap_agent
                            ok = 0;
                        end
                    end

                end
                % Randomly distribute agents as a skewed circle
            case 2
                if abs(sum((a_location(i,:)))) < a_location_limit(1) % Case (2) Skewed Circle
                    ok = 1;
                    for k=1:i-1
                        if norm(a_location(k,:) - a_location(i,:)) < Gap_agent
                            ok = 0;
                        end
                    end

                end
                % Randomly distribute agents as a square
            case 3
                ok = 1;
                for k=1:i-1
                    if norm(a_location(k,:) - a_location(i,:)) < Gap_agent
                        ok = 0;
                    end
                end
        end

    end
end

% Generation Agent Communication (Neighbour agents within communication
% radius)
dist_agents = zeros(n,n);
for i=1:n
    for j=1:n
        dist_agents(i,j) = norm(a_location(i,:)-a_location(j,:));
    end
end

% Neighbour agents within communication radius
MST_ = (dist_agents <= Comm_distance);
MST = MST_ - eye(n,n);
% Note: MST will be used in Task_Allocation.m (Task_Allocation_SC_visual.m) to simulate communications between agents

%%%% MODIFIED PART STARTS HERE %%%%
%% Phase 1 Grouping : Leader-follower grouping based on paper (Modifed 17.Jan.2026)

%  - K   : 최대 follower-to-leader 비 (논문에서 사용하는 K, 예: 3)

K = 12;    % 필요하면 바꿔서 실험 - 11 이상부터 모든 leader에 task가 할당됨

%[1] Initialize L and F
L = [];             % leaders 집합 (agent index)
F = (1:n).';        % followers 집합 (초기에는 전체 agent)

%[2] Compute degree of each agent
deg_agents = sum(MST, 2);    % 각 행의 합 = degree (이웃 수)

%[3-6] GPI 방식의 leader 선택
%    Candidate follower agents are sequentially chosen as leaders until
%    (i) 모든 follower가 최소 한 명의 leader와 1-hop 이웃,
%    (ii) follower-to-leader 비가 K 이하.

%[3]  GPI condition 2개가 만족될 때까지 leader 선택 반복
while true
    numL = numel(L); %Leader 수
    numF = numel(F); %Follower 수

    % (i) coverage 조건: 모든 follower가 최소 한 leader와 1-hop 이웃인지
    if numL == 0
        all_covered = false;
    else
        % MST(F, L) : follower vs leader adjacency
        covered = any(MST(F, L) > 0, 2);   % 각 follower가 어떤 leader와라도 연결되어 있으면 true
        all_covered = all(covered);
    end

    % (ii) follower-to-leader 비 계산
    if (numL > 0) && all_covered && (numF / numL <= K)
        % numL > 0 이라서 0으로 나누는 경우는 없음
        break;
    end


    % [4] 새 leader 선택
    if numL == 0
        % L = ∅ 이면 F 전체가 candidate (h ≡ 1 인 경우)
        cand = F;
    else
        % leader 와 1-hop 이웃인 follower 들만 1차 후보 (h = 1)
        cand_mask = any(MST(F, L) > 0, 2);   % F 안에서 leader 와 직접 연결된 애들
        if any(cand_mask)
            cand = F(cand_mask);
        else
            % 모든 follower 가 leader 와 1-hop 이웃이 아니면
            % h = 0 이라 d_u·h = 0 이지만, argmax 는 여전히 F 전체에서 뽑는다.
            cand = F;
        end
    end

    % 더 이상 isempty(cand)로 break 하지 않는다. 후보들 중에 degree 최대인 애 선택해 새 leader 로
    [~, idx_max] = max(deg_agents(cand));
    new_leader = cand(idx_max);

    %[5] 새 leader 추가 후 follower 집합에서 제거
    L = [L; new_leader];
    F(F == new_leader) = [];

end

% 최종 leaders / followers (정렬해 두면 보기 편함)
leaders   = sort(L(:).');
followers = sort(F(:).');

% 논리 벡터
isLeader   = false(n,1);
isFollower = false(n,1);
isLeader(leaders)     = true;
isFollower(followers) = true;

%% [7~11] Followers -> Leaders 그룹 배정 (최소 group_size 리더 선택)

%[7] 각 group 정보 초기화
num_leaders        = numel(leaders);
group_size         = zeros(num_leaders,1);   % 각 리더 밑 follower 수
follower_to_leader = zeros(n,1);            % f 가 어느 리더(leaders(k))에 붙었는지
groups             = cell(num_leaders,1);   % groups{k} = leaders(k) 밑 follower 리스트

%[8] 각 follower 에 대해
for f = followers
    %[9]-1)min distance =1 ; 이 follower f와 MST로 연결된 leader들만 후보
    cand_idx = find(MST(f, leaders) > 0);   % leaders 중 edge가 있는 리더들의 서브 인덱스

    if isempty(cand_idx) %Alg 1에 없는 내용; gpi 조건 안 맞는 경우의 처리
        % follower는 항상 적어도 한 leader와 1-hop이어야 함
        continue;
    end

    %[9]-2) Arg min = k; 후보 리더들 중 현재 group_size가 최소인 리더 선택 (load balancing)
    [~, pos] = min(group_size(cand_idx));
    k = cand_idx(pos);                      % leaders(k)가 실제 리더 번호

    % [10] 그룹에 follower 추가
    group_size(k)         = group_size(k) + 1;
    follower_to_leader(f) = leaders(k);
    groups{k}             = [groups{k}, f];
end

%% (이론상 비어 있어야 하는) Unassigned follower 확인
unassigned_followers = followers(follower_to_leader(followers) == 0);

%% 확인용 출력
fprintf('===== Leader / Follower grouping result (GPI) =====\n');
fprintf('Total agents: %d\n', n);
fprintf('Num leaders : %d\n', numel(leaders));
fprintf('Num followers: %d\n', numel(followers));
fprintf('Follower-to-Leader ratio = %.2f (target K = %.2f)\n', ...
    numel(followers)/max(numel(leaders),1), K);
fprintf('\n');

fprintf('Leaders   = (%s)\n',   join(string(leaders), ', '));
fprintf('Followers = (%s)\n\n', join(string(followers), ', '));

fprintf('=== Leader-wise follower groups ===\n');
for k = 1:num_leaders
    fprintf('Leader %d | followers(%d) = (', leaders(k), group_size(k));
    if ~isempty(groups{k})
        fprintf('%s', join(string(groups{k}), ', '));
    end
    fprintf(')\n');
end

if ~isempty(unassigned_followers)
    fprintf('Unassigned followers (no connected leader, after grouping) = (%s)\n', ...
        join(string(unassigned_followers), ', '));
end
fprintf('=============================================\n\n');

%index 출력; 높은 degree 순서대로, degree 동일할 경우 index 작은 순서대로
fprintf('Index of Leaders   = (%s)\n', join(string(leaders), ', '));
fprintf('Index of Followers = (%s)\n', join(string(followers), ', '));
fprintf('\n');

%%확인2: agent 별 degree 출력
% fprintf('Leaders (index, degree):\n');
% for k = 1:num_leaders
%     i = leaders(k);
%     fprintf('  agent %d  | degree = %d\n', i, deg_agents(i));
% end
% fprintf('\n');

% fprintf('Followers (index, degree):\n');
% for k = 1:numel(followers)
%     i = followers(k);
%     fprintf('  agent %d  | degree = %d\n', i, deg_agents(i));
% end
% fprintf('=============================================\n');

%%%%Task 1 Grouping : MODIFIED PART ENDS HERE%%%%

%
%

%%%%Task 2 Task Allocation : MODIFIED PART STARTS HERE%%%%

%% Initialise task allocation & Merge and Split Algorithm

%% =======================
%% Phase 1) Task Grouping (random, group size 비례)
%% =======================
rng(0);  % 재현성 필요 없으면 삭제

numL = numel(leaders);     % leaders: 1 x numL (또는 numL x 1)
leaders = leaders(:);      % column으로 통일

% 각 leader-group의 agent 수(g_k) = leader 1명 + follower 수
group_member_cnt = zeros(numL,1);
for k = 1:numL
    group_member_cnt(k) = 1 + numel(groups{k});
end

% m개 task를 group_member_cnt 비율로 나눔 (T/L)
w = group_member_cnt / sum(group_member_cnt);
mk_float = w * m;
mk = floor(mk_float);
rem = m - sum(mk);

% 나머지는 소수점 큰 순서로 분배 (논문에 없지만 추가한 규칙)
[~, order] = sort(mk_float - mk, 'descend');
mk(order(1:rem)) = mk(order(1:rem)) + 1;

% task index 랜덤 셔플 후, mk만큼 잘라서 그룹 생성 (random task groups)
perm_tasks = randperm(m);
task_groups = cell(numL,1);
idx = 1;
for k = 1:numL
    task_groups{k} = perm_tasks(idx : idx + mk(k) - 1);
    idx = idx + mk(k);
end

%% =======================
%% Phase 2) Task 그룹을 Leader에게 random 할당
%% =======================
perm_bundle = randperm(numL); % random indice of leaders
leader_tasks = cell(numL,1);      % leader_tasks{k}: leaders(k)가 맡을 task의 indice - random하게 할당됨
for k = 1:numL
    leader_tasks{k} = task_groups{perm_bundle(k)};
end

%% =======================
%% Debug/Log) Task grouping & assignment 확인
%% =======================

fprintf('\n===== Task Grouping (task_groups) =====\n');
fprintf('m = %d, numL = %d\n', m, numL);
fprintf('mk (tasks per group, before leader mapping) = ');
fprintf('%d ', mk);
fprintf('\n\n');

for k = 1:numL
    tg = task_groups{k};
    fprintf('Group %d | tasks(%d) = (', k, numel(tg));
    if ~isempty(tg)
        fprintf('%s', strtrim(sprintf('%d ', tg)));
    end
    fprintf(')\n');
end

fprintf('===== Task -> Leader assignment (leader_tasks) =====\n');

task_to_leader = zeros(m,1);   % task t가 배정된 leader index (없으면 0)
for k = 1:numL
    tk = leader_tasks{k};
    if ~isempty(tk)
        task_to_leader(tk) = leaders(k);
    end
end

for k = 1:numL
    tk = leader_tasks{k};
    fprintf('Leader %d | tasks(%d) = (', leaders(k), numel(tk));
    if ~isempty(tk)
        fprintf('%s', strtrim(sprintf('%d ', tk)));
    end
    fprintf(')\n');
end

fprintf('----- Per-task summary -----\n');
for t = 1:m
    fprintf('Task %d -> Leader %d\n', t, task_to_leader(t));
end
fprintf('=======================================\n\n');


%% =======================
%% Phase 2-2) Group-wise Task Allocation
%% (Task_Allocation_SC_visual을 그룹별로 여러 번 호출)
%% =======================
Alloc = zeros(n,1);
a_utility = zeros(n,1);
iteration = zeros(numL,1);
flag_problem = zeros(numL,1);

for k = 1:numL
    members = [leaders(k); groups{k}(:)];  % 전역 agent index
    tasks_k = leader_tasks{k}(:);          % 전역 task index

    if isempty(tasks_k)  % 이 그룹에 배정된 task가 0개면 스킵
        Alloc(members) = 0;
        a_utility(members) = 0;
        iteration(k) = 0;
        flag_problem(k) = 0;
        continue;
    end

    nk = numel(members);
    mk_local = numel(tasks_k);

    % 부분 환경 구성 (agent/ task subset)
    env_k.a_location = a_location(members,:);
    env_k.t_location = t_location(tasks_k,:);
    env_k.t_demand   = t_demand(tasks_k);

    % 부분 입력 구성
    input_k.Alloc_existing = zeros(nk,1);
    input_k.Flag_display = Flag_display;
    input_k.n = nk;
    input_k.m = mk_local;
    input_k.environment = env_k;

    % 그룹 내부 통신: fully connected (현재 Comm_distance=300의 의도와 동일)
    input_k.MST = ones(nk,nk) - eye(nk);

    out_k = Task_Allocation_SC_visual(input_k);

    % out_k.Alloc은 1..mk_local(부분 task index)
    alloc_local = out_k.Alloc;          % nk x 1
    alloc_global = zeros(nk,1);
    mask = (alloc_local > 0);
    alloc_global(mask) = tasks_k(alloc_local(mask));  % 전역 task index로 매핑

    Alloc(members) = alloc_global;
    a_utility(members) = out_k.a_utility;
    iteration(k) = out_k.iteration;
    flag_problem(k) = out_k.flag_problem;
end

% Alloc_existing = zeros(n,1);    % Initial task assignment: every robot is assigned to void task
%
%
% input.Alloc_existing = Alloc_existing;
% input.Flag_display = Flag_display;
% input.MST = MST;
% input.n = n;
% input.m = m;
% input.environment = environment;
% %
% %%%% Method (1): All Agents are deployed at once
% [output] = Task_Allocation_SC_visual(input); % Consiering Strongly-connected environment
% % Output : Alloc / a_utility / iteration
% %
% Alloc = output.Alloc;
% a_utility = output.a_utility;
% iteration = output.iteration;
% flag_problem = output.flag_problem; % If the result has a problem, then 1.
%
environment.t_location = t_location;
environment.t_demand = t_demand;
environment.a_location = a_location;
%
%% Minimum-guaranteed Global Utility (Theorem 3)
Minimum_Guaranteed_Optimality;

