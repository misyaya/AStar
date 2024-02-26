#include "AStar.h"

int main()
{
	// マップの幅設定
	const int width = 10; 

	// マップの高さ設定
	const int height = 10;

	// スタートXY座標
	const int startX = 0;
	const int startY = 0;

	// ゴールXY座標
	const int goalX = 9;
	const int goalY = 9;

	// インスタンス作成
	AStar aStar(width, height);

	// スタート設定
	aStar.SetStart(startX, startY);

	// ゴール設定
	aStar.SetGoal(9, 9);

	// 乱数のシードを設定
	srand(static_cast<unsigned>(time(nullptr)));  

	// 障害物設定
	for(int i = 0; i < 25; i++)
	{
		int x = rand() % width;
		int y = rand() % height;

		if ((x != startX || y != startY) && (x != goalX || y != goalY))
		{
			aStar.SetObstacle(x, y);
		}
	}

	// 経路探索
	vector<Node*> path = aStar.FindPath();

	// パスが見つかったか
	if (!path.empty())
	{
		cout << "Path found: ";

		for (int i = path.size() - 1; i >= 0; --i)
		{
			cout << "(" << path[i]->x << "," << path[i]->y << ") ";
		}
		cout << endl;
	}
	else
	{
		cout << "Path not found!" << endl;
	}

	cout << "\n" << "□:通路  ■:障害物  〇:最短経路" << endl;

	cout << "\n" <<  "～Map～" << endl;

	aStar.PrintMap();

	return 0;


}