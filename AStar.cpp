#include "AStar.h"

AStar::AStar(int _width, int _height) 
	: width_(_width), height_(_height)
{
	// nodes_初期化　width_行、height_列の2次元ベクター

	// nullptrで初期化
	nodes_.resize(width_, vector<Node*>(height_, nullptr));

	// 座標を持たせる
	for (int x = 0; x < width_; ++x)
	{
		for (int y = 0; y < height_; ++y)
		{
			nodes_[x][y] = new Node(x, y);
		}
	}
}

void AStar::SetObstacle(int _x, int _y)
{
	if (IsValid(_x, _y))
	{
		// nodes_には基本的に座標が入っているため nullptr->障害物ありと判定する
		nodes_[_x][_y] = nullptr;
	}
}

void AStar::SetGoal(int _x, int _y)
{
	if (IsValid(_x, _y))
	{ 
		// ゴール位置を入れる
		goal_ = nodes_[_x][_y];
	}
}

bool AStar::IsValid(int _x, int _y) const
{
	// 与えられた値がマップの範囲内か、障害物の有無を調べる
	return (_x >= 0) && (_x < width_) && (_y >= 0) && (_y < height_) && (nodes_[_x][_y] != nullptr);
}

bool AStar::IsObstacle(int _x, int _y) const
{
	// IsValidからfalseがくる＝障害物がある
	return !IsValid(_x, _y);
}

vector<Node*> AStar::FindPath()
{
	openList_.push_back(start_);

	while (!openList_.empty())
	{
		// オープンリスト内で最小のfコストを持つノードをcurrentに入れる
		Node* current = openList_[0];

		// 最小のfコストのオープンリスト内での位置を入れる
		int currentIndex = 0;

		for (int i = 1; i < openList_.size(); ++i)
		{
			if (openList_[i]->fCost() <  current->fCost() ||
				openList_[i]->fCost() == current->fCost() &&
				openList_[i]->hCost   <  current->hCost)
			{
				current = openList_[i];
				currentIndex = i;
			}
		}

		// ゴールに到達した場合、パスを生成して返す
		if (current == goal_)
		{
			// 最短経路が入る
			vector<Node*> path;

			// スタート位置に戻るまで繰り返す(ゴールから逆順にたどっていく)
			while (current != nullptr)
			{
				path.push_back(current);

				// currentがどこから来たのかをたどる
				current = current->parent;
			}
			return path;
		}

		// オープンリストから選択したノードを削除、クローズドリストに追加
		openList_.erase(openList_.begin() + currentIndex);
		closeList_.push_back(current);
	
		// 隣接するノードを調べる 左右上下、斜め四方向
		vector<Node*> neighbors;

		for (int i = 0; i < direction_; ++i)
		{
			int newX = current->x + dx[i];
			int newY = current->y + dy[i];

			// マップの範囲内、クローズリストに入っていない(まだ調べてない)か確認　入っていなければfindが最後まで探すためcloseList.end()が返ってくる
			if (IsValid(newX, newY) && find(closeList_.begin(), closeList_.end(), nodes_[newX][newY]) == closeList_.end())
			{
				neighbors.push_back(nodes_[newX][newY]);
			}
		}


		// 隣接するノードに対して処理を行う
		for (Node* neighbor : neighbors)
		{
			// gコスト(開始地点からのコスト)を計算
			int newGCost = current->gCost + 1;

			if (newGCost < neighbor->gCost || 
				find(openList_.begin(), openList_.end(), neighbor) == openList_.end())
			{
				neighbor->gCost = newGCost;
				neighbor->hCost = heuristic(neighbor, goal_);
				neighbor->parent = current;

				if (find(openList_.begin(), openList_.end(), neighbor) == openList_.end())
				{
					openList_.push_back(neighbor);
				}
			}
		}
	}
	
	// ゴールに到達できなかった場合、空のパスを返す
	return vector<Node*>();
}