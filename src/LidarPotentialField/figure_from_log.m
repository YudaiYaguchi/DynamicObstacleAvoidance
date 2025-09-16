% データ（Time(s) と Distance(m)）
time = (0:7)';  % 0～7秒
dist = [3.90; 3.39; 2.90; 2.39; 1.90; 1.39; 0.89; 0.40];

% テーブル化
T = table(time, dist, 'VariableNames', {'Time_s','Distance_m'});

% Excel ファイルに書き出し
filename = 'distance_log.xlsx';
writetable(T, filename);
fprintf('Saved data to %s\n', filename);

% グラフ作成
figure;
plot(T.Time_s, T.Distance_m, '-o','LineWidth',2);
grid on;
xlabel('時間 (s)');
ylabel('目的地との距離 (m)');
% title('障害物との距離の変化');
% 終了後10秒間ウィンドウを保持
pause(40);

