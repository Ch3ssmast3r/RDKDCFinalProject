function [] = tictactoefast()
%% Teach and Make the Grid
ur5 = ur5_interface(); % starting the ur5 interface

starting_config = [0.8996; -1.7011; 2.4081; -2.2778; -1.5627; 0.1232];

% only moving to start configuration if we are not near it. 
if (norm(ur5.get_current_joints() - starting_config) > 0.1)
    ur5.switch_to_ros_control();
    disp('Moving to starting_config');
    ur5.move_joints(starting_config, 15);
    pause(15)
end

% teach the points we want to go to. 
ur5.switch_to_pendant_control();
frames = ur5_teach_points(ur5);
ur5.switch_to_ros_control();
[x,y, l, g0] = ur5_calcgridparameters(frames(:,:,1), frames(:,:,2));
physical_grid = calc_grid(g0(1:3, 4), x, y, l);

disp('moving up');
up_displacement = zeros(4);
up_displacement(3, 4) = 0.02;
ur5FwdKinDH(ur5.get_current_joints);
up_frame = ur5FwdKinDH(ur5.get_current_joints()) + up_displacement;
ur5RRcontrolSmooth(up_frame, ur5);

% draw_list(g0, physical_grid, ur5);
centers = calc_centers(g0(1:3,4), x, y, l);
x0_size = calc_shape_size(l);

grid_params = GridParams(g0, x, y, l, centers, x0_size, ur5);
%% Set up Tic Tac Toe
    % Setup the figure/windows for the game
    close all
    figure('Name','Tic Tac Toe');
    plot(-1. -1)
    axis([0 3 0 3])
    set(gca,'xTick',0:3)
    set(gca,'yTick',0:3)
    set(gca,'xTickLabel','')
    set(gca,'yTickLabel','')
    xlabel('Player: X')
    grid on
    shg
    
    is_x = 1; % keeps track of the current player
    state = [[-1 -1 -1]
             [-1 -1 -1]
             [-1 -1 -1]]; % the state of the game (-1 none, 0 = O, 1 = X)
    winner = -1; % is there a winner? is it a tie?
%% Main Game Loop
    % The main game loop. Continue until the game ends with winner ~= -1
    while winner == -1
        next = play(is_x, state, grid_params); % play a single round
        if next == -1 % if the player clicks on a filled in slot, ask them to try again
           title('Invalid move, please try again');
        else
           state = next; % advance the current state
           title('');
           is_x = mod(is_x + 1,2); % pick the next player and update the player label
           if is_x == 1
               xlabel('Player: X');
           else
               xlabel('Player: O');
           end
           winner = won(state, grid_params); % check to see if the game is in a winning state
        end
    end
    
    if winner == 0 % O won
        ur5RRcontrolSmooth(ur5FwdKinDH(starting_config), ur5);
        warndlg('O wins');
        title('O wins');
        xlabel('');
    elseif winner == 1 % X won
        warndlg('X wins');
        title('X wins');
        xlabel('');
        ur5RRcontrolSmooth(ur5FwdKinDH(starting_config), ur5);
    else % else it's a tie
        warndlg('Tie');
        title('Tie');
        xlabel('');
        ur5RRcontrolSmooth(ur5FwdKinDH(starting_config), ur5);
    end
end

% The state function takes in the current player and the previous state the
% game is in and simulates a single round.
function state = play(is_x, state, grid_params)
    [x, y] = ginput(1); % get the mouse position with respect to the plot
    [col, row] = position(x, y); % get the corresponding row/col (note row starts off with 0 at the bottom)
    %assign each row, column to the correct center point (from
    %calc_centers)
    row = 2 - row; % the actual row within the state matrix
    index = 3*row + (col + 1);
    if state(col+1, row+1) ~= -1 % if the player tries to click on a filled spot
       state = -1; % invalid, ask the player to try again
    else
        state(col+1, row+1) = is_x; % set the state and draw the X and the O
        if is_x
            drawX(col, 2 - row, index, grid_params);
        else
            drawO(col, 2 - row, index, grid_params);
        end
    end
end

% The won function calculates if the current game state is in a winning
% state.
function won = won(state, grid_params)
    centers = grid_params.centers;
    g0      = grid_params.g0;
    ur5 = grid_params.ur5;
    % Horizontal
    if (state(1,1) == state(1,2) && state(1,1) == state(1,3) && state(1,1) ~= -1)
        won = state(1,1);
        win_line = calc_win_line(centers(:,1), centers(:,7));
        draw_list(g0, win_line, ur5);
    elseif (state(2,1) == state(2,2) && state(2,1) == state(2,3) && state(2,1) ~= -1)
        won = state(2,1);
        win_line = calc_win_line(centers(:,2), centers(:,8));
        draw_list(g0, win_line, ur5);
    elseif (state(3,1) == state(3,2) && state(3,1) == state(3,3) && state(3,1) ~= -1)
        won = state(3,1);
        win_line = calc_win_line(centers(:,3), centers(:,9));
        draw_list(g0, win_line, ur5);
    % Vertical
    elseif (state(1,1) == state(2,1) && state(1,1) == state(3,1) && state(3,1) ~= -1) 
        won = state(1,1);
        win_line = calc_win_line(centers(:,1), centers(:,3));
        draw_list(g0, win_line, ur5);
    elseif (state(1,2) == state(2,2) && state(1,2) == state(3,2) && state(1,2) ~= -1) 
        won = state(1,2);
        win_line = calc_win_line(centers(:,4), centers(:,6));
        draw_list(g0, win_line, ur5);
    elseif (state(1,3) == state(2,3) && state(1,3) == state(3,3) && state(1,3) ~= -1) 
        won = state(1,3);
        win_line = calc_win_line(centers(:,7), centers(:,9));
        draw_list(g0, win_line, ur5);
    % Diagonal
    elseif (state(1,1) == state(2,2) && state(1,1) == state(3,3) && state(1,1) ~= -1)
        won = state(1,1);
        win_line = calc_win_line(centers(:,1), centers(:,9));
        draw_list(g0, win_line, ur5);
    elseif (state(1,3) == state(2,2) && state(1,3) == state(3,1) && state(2,2) ~= -1)
        won = state(1,3);
        win_line = calc_win_line(centers(:,3), centers(:,7));
        draw_list(g0, win_line, ur5);
    % If no more slots are open, it's a tie
    elseif ~ismember(state, -1)
        won = 2;
    else
        won = -1;
    end
end

% Returns the rounded off position of the mouse
function [col, row] = position(x, y)
    col = floor(x);
    row = floor(y);
    if col > 2 % if we're right on the 3 line, we count it as 2
        col = 2;
    end
    
    if row > 2
        row = 2;
    end

end

function drawX(col, row, index, grid_params)
    hold on
    x = 0:1;
    pos = 0:1;
    neg = 1-x;
    plot(x+col, pos+row)
    plot(x+col, neg+row)
    hold off
    centers = grid_params.centers;
    x0_size = grid_params.x0_size;
    cross_x = grid_params.x;
    cross_y = grid_params.y;
    g0      = grid_params.g0;
    ur5 = grid_params.ur5;
    cross = calc_cross(centers(:, index), x0_size, cross_x, cross_y);
    % draw_list(g0, cross, ur5);
end

function drawO(col, row, index, grid_params)
    hold on
    t = 0:0.1:2*pi;
    x = cos(t)/2+0.5;
    y = sin(t)/2+0.5;
    plot(x+col, y+row)
    hold off   
    centers = grid_params.centers;
    x0_size = grid_params.x0_size;
    g0      = grid_params.g0;
    ur5 = grid_params.ur5;
    circle = calc_circle(centers(:, index), x0_size);
    % draw_list(g0, circle, ur5);
end