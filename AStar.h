#ifndef ASTAR_H // �ǂݍ��ݏd�����
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
	int x; // �m�[�h��X���W
	int y; // �m�[�h��Y���W
	int gCost; // �J�n�n�_����̃R�X�g
	int hCost; // �S�[���܂ł̐���R�X�g
	Node* parent; // �e�m�[�h�ւ̃|�C���^
	bool isOnPath = false; // �p�X��̃m�[�h

	//�R���X�g���N�^�@���ꂼ��̒l�̏�����
	Node(int _x, int _y):
		x(_x), y(_y), gCost(0), hCost(0), parent(nullptr){}
	
	// fCost�̌v�Z�@�@fCost->����m�[�h�ł̃g�[�^���R�X�g�̂��� 
	int fCost() const 
	{
		return gCost + hCost;
	}

};

class AStar {
private:
	int width_; // �}�b�v�̕�
	int height_; // �}�b�v�̍���

	Node* start_; // �X�^�[�g�ʒu
	Node* goal_; // �S�[���ʒu


	static const int direction_ = 8; // �אڃm�[�h�T���͈́@�W����
	int dx[direction_] = { -1, 0, 1, -1, 1, -1, 0, 1 };
	int dy[direction_] = { -1, -1, -1, 0, 0, 1, 1, 1 };


	vector<vector<Node*>> nodes_; // �m�[�h�̓񎟌��z��
	vector<Node*> openList_; // �I�[�v�����X�g
	vector<Node*> closeList_; // �N���[�Y���X�g

	Node* GetNode(int x, int y) const; // ���W�̃m�[�h���擾
	void CalculateCosts(Node* node); // �R�X�g�̌v�Z
	int Heuristic(Node* a, Node* b) const; // �q���[���X�e�B�b�N�֐�

public:
	//�R���X�g���N�^
	AStar(int _width, int _height);

	// ��Q���̐ݒu
	void SetObstacle(int _x, int _y); 
	
	// �X�^�[�g�ʒu�̐ݒ�
	void SetStart(int _x, int _y); 

	// �S�[���ʒu�̐ݒ�
	void SetGoal(int _x, int _y); 

	// �}�b�v�͈͓̔����ǂ������m�F
	bool IsValid(int _x, int _y) const; 

	// ���W����Q�����ǂ������m�F
	bool IsObstacle(int _x, int _y) const; 
	
	// �p�X��������
	vector<Node*> FindPath();

	void PrintMap() const;

};

#endif // ASTAR_H