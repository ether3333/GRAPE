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
t_location_limit = [500 500]; % Task locations range: max(X) max(Y) (metre)
a_location_limit = [300 300]; % Agent locations range: max(X) & max(Y) (metre)

% Task Demand or Reward
t_demand_mean = 1000*n/m; % 15.(Jul.2016) In order to maintain the level of individual utilities regardless of #Tasks or #Agents

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
t_location = zeros(m,2);    % task location - 2D
a_location = zeros(n,2);    % agent location - 2D
t_demand = zeros(m,1);      % task rewards

Comm_distance = 50;         % Communication range of each robot
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
%% Phase 1 Grouping : Leader-follower grouping based on paper (Modifed 26.Nov.2025)

%  - K   : 최대 follower-to-leader 비 (논문에서 사용하는 K, 예: 3)

K = 3;    % 필요하면 바꿔서 실험

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

%%%%MODIFIED PART ENDS HERE%%%%
%%%%주석 처리하라고 하셨던 부분 － 없으니까 안 돌아가서 복원함%%%%

environment.t_location = t_location;
environment.t_demand = t_demand;
environment.a_location = a_location;
%
%
%% Initialise task allocation & Merge and Split Algorithm
Alloc_existing = zeros(n,1);    % Initial task assignment: every robot is assigned to void task
%
%
input.Alloc_existing = Alloc_existing;
input.Flag_display = Flag_display;
input.MST = MST;
input.n = n;
input.m = m;
input.environment = environment;
%
%%%% Method (1): All Agents are deployed at once
[output] = Task_Allocation_SC_visual(input); % Consiering Strongly-connected environment
% Output : Alloc / a_utility / iteration
%
Alloc = output.Alloc;
a_utility = output.a_utility;
iteration = output.iteration;
flag_problem = output.flag_problem; % If the result has a problem, then 1.
%
%
%% Minimum-guaranteed Global Utility (Theorem 3)
Minimum_Guaranteed_Optimality;

