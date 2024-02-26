#include "AStar.h"

AStar::AStar(int _width, int _height) 
	: width_(_width), height_(_height)
{
	// nodes_�������@width_�s�Aheight_���2�����x�N�^�[

	// nullptr�ŏ�����
	nodes_.resize(width_, vector<Node*>(height_, nullptr));

	// ���W����������
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
		// nodes_�ɂ͊�{�I�ɍ��W�������Ă��邽�� nullptr->��Q������Ɣ��肷��
		nodes_[_x][_y] = nullptr;
		
	}
}

void AStar::SetStart(int _x, int _y)
{
	// �X�^�[�g�ʒu�������
	if (IsValid(_x, _y))
	{
		start_ = nodes_[_x][_y];
	}
}

void AStar::SetGoal(int _x, int _y)
{
	if (IsValid(_x, _y))
	{ 
		// �S�[���ʒu������
		goal_ = nodes_[_x][_y];
	}
}

bool AStar::IsValid(int _x, int _y) const
{
	// �^����ꂽ�l���}�b�v�͈͓̔����A��Q���̗L���𒲂ׂ�
	return (_x >= 0) && (_x < width_) && (_y >= 0) && (_y < height_) && (nodes_[_x][_y] != nullptr);
}

bool AStar::IsObstacle(int _x, int _y) const
{
	// IsValid����false�����遁��Q��������
	return !IsValid(_x, _y);
}

vector<Node*> AStar::FindPath()
{
	openList_.push_back(start_);

	while (!openList_.empty())
	{
		// �I�[�v�����X�g���ōŏ���f�R�X�g�����m�[�h��current�ɓ����
		Node* current = openList_[0];

		// �ŏ���f�R�X�g�̃I�[�v�����X�g���ł̈ʒu������
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

		// �S�[���ɓ��B�����ꍇ�A�p�X�𐶐����ĕԂ�
		if (current == goal_)
		{
			// �ŒZ�o�H������
			vector<Node*> path;

			// �X�^�[�g�ʒu�ɖ߂�܂ŌJ��Ԃ�(�S�[������t���ɂ��ǂ��Ă���)
			while (current != nullptr)
			{
				path.push_back(current);
				
				// �ŒZ�o�H�̃m�[�h
				current->isOnPath = true;

				// current���ǂ����痈���̂������ǂ�
				current = current->parent;

			}
			return path;
		}

		// �I�[�v�����X�g����I�������m�[�h���폜�A�N���[�Y�h���X�g�ɒǉ�
		openList_.erase(openList_.begin() + currentIndex);
		closeList_.push_back(current);
	
		// �אڂ���m�[�h�𒲂ׂ� ���E�㉺�A�΂ߎl����
		vector<Node*> neighbors;

		for (int i = 0; i < direction_; ++i)
		{
			int newX = current->x + dx[i];
			int newY = current->y + dy[i];

			// �}�b�v�͈͓̔��A�N���[�Y���X�g�ɓ����Ă��Ȃ�(�܂����ׂĂȂ�)���m�F�@�����Ă��Ȃ����find���Ō�܂ŒT������closeList.end()���Ԃ��Ă���
			if (IsValid(newX, newY) && find(closeList_.begin(), closeList_.end(), nodes_[newX][newY]) == closeList_.end())
			{
				neighbors.push_back(nodes_[newX][newY]);
			}
		}


		// �אڂ���m�[�h�ɑ΂��ď������s��
		for (Node* neighbor : neighbors)
		{
			// g�R�X�g(�J�n�n�_����̃R�X�g)���v�Z
			int newGCost = current->gCost + 1;

			if (newGCost < neighbor->gCost || 
				find(openList_.begin(), openList_.end(), neighbor) == openList_.end())
			{
				neighbor->gCost = newGCost;
				neighbor->hCost = Heuristic(neighbor, goal_);
				neighbor->parent = current;

				if (find(openList_.begin(), openList_.end(), neighbor) == openList_.end())
				{
					openList_.push_back(neighbor);
				}
			}
		}
	}
	
	// �S�[���ɓ��B�ł��Ȃ������ꍇ�A��̃p�X��Ԃ�
	return vector<Node*>();
}

Node* AStar::GetNode(int x, int y) const
{
	if (IsValid(x, y))
	{
		//�w�肳�ꂽ�m�[�h���͈͓��Ȃ�Ԃ�
		return nodes_[x][y];
	}
	
	return nullptr;
}

void AStar::CalculateCosts(Node* node)
{
	if (node != nullptr)
	{
		//�X�^�[�g�m�[�h����ǂꂾ������Ă��邩
		node->gCost = abs(node->x - start_->x) + abs(node->y - start_->y);
		
		//�S�[���m�[�h����ǂꂾ������Ă��邩
		node->hCost = abs(node->x - goal_->x) + abs(node->y - goal_->y);
	}
}

int AStar::Heuristic(Node* a, Node* b) const
{
	//����R�X�g�v�Z
	return abs(a->x - b->x) + abs(a->y - b->y);
}

void AStar::PrintMap() const 
{
	// ��:�ʘH  ��:��Q��  �Z:�ŒZ�o�H
	for (int y = 0; y < height_; ++y) 
	{
		for (int x = 0; x < width_; ++x) 
		{
			if (nodes_[x][y] != nullptr) 
			{
				// �ŒZ�o�H��������Z
				if (nodes_[x][y]->isOnPath)
				{
					cout << "�Z ";
				}
				else
				{
					cout << "�� ";
				}
			}
			else 
			{
				cout << "�� ";
			}
		}
		cout << endl;
	}
}