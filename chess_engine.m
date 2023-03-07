%% chess engine access %%

% Start Rybka
[status,result] = system('./stockfish_15.1_win_x64_avx2/src/stockfish-windows-2022-x86-64-avx2.exe');

% Send UCI command
system('uci');

% Read UCI output
output = result;