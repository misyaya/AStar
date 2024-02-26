#ifndef ASTAR_H // 読み込み重複回避
#define ASTAR_H

#include <iostream>
#include <vector>
#include <queue>
#include <cmath>

using std::cin;
using std::cout;
using std::endl;
using std::vector;
using std::find;
using std::abs;

struct Node
{
	int x; // ノードのX座標
	int y; // ノードのY座標
	int gCost; // 開始地点からのコスト
	int hCost; // ゴールまでの推定コスト
	Node* parent; // 親ノードへのポインタ
	bool isOnPath = false; // パス上のノード

	//コンストラクタ　それぞれの値の初期化
	Node(int _x, int _y):
		x(_x), y(_y), gCost(0), hCost(0), parent(nullptr){}
	
	// fCostの計算　　fCost->あるノードでのトータルコストのこと 
	int fCost() const 
	{
		return gCost + hCost;
	}

};

class AStar {
private:
	int width_; // マップの幅
	int height_; // マップの高さ

	Node* start_; // スタート位置
	Node* goal_; // ゴール位置


	static const int direction_ = 8; // 隣接ノード探索範囲　８方向
	int dx[direction_] = { -1, 0, 1, -1, 1, -1, 0, 1 };
	int dy[direction_] = { -1, -1, -1, 0, 0, 1, 1, 1 };


	vector<vector<Node*>> nodes_; // ノードの二次元配列
	vector<Node*> openList_; // オープンリスト
	vector<Node*> closeList_; // クローズリスト

	Node* GetNode(int x, int y) const; // 座標のノードを取得
	void CalculateCosts(Node* node); // コストの計算
	int Heuristic(Node* a, Node* b) const; // ヒューリスティック関数

public:
	//コンストラクタ
	AStar(int _width, int _height);

	// 障害物の設置
	void SetObstacle(int _x, int _y); 
	
	// スタート位置の設定
	void SetStart(int _x, int _y); 

	// ゴール位置の設定
	void SetGoal(int _x, int _y); 

	// マップの範囲内かどうかを確認
	bool IsValid(int _x, int _y) const; 

	// 座標が障害物かどうかを確認
	bool IsObstacle(int _x, int _y) const; 
	
	// パスを見つける
	vector<Node*> FindPath();

	void PrintMap() const;

};

#endif // ASTAR_H