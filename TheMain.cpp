#include "AStar.h"

int main()
{
	//マップの幅設定
	const int width = 10; 

	//マップの高さ設定
	const int height = 10;

	// インスタンス作成
	AStar aStar(width, height);

	// スタート設定
	aStar.SetStart(0, 0);

	// ゴール設定
	aStar.SetGoal(9, 9);

	// 障害物設定
	aStar.SetObstacle(2, 2);
	aStar.SetObstacle(3, 3);
	aStar.SetObstacle(4, 4);

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

	return 0;


}