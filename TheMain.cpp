#include "AStar.h"

int main()
{
	// �}�b�v�̕��ݒ�
	const int width = 10; 

	// �}�b�v�̍����ݒ�
	const int height = 10;

	// �X�^�[�gXY���W
	const int startX = 0;
	const int startY = 0;

	// �S�[��XY���W
	const int goalX = 9;
	const int goalY = 9;

	// �C���X�^���X�쐬
	AStar aStar(width, height);

	// �X�^�[�g�ݒ�
	aStar.SetStart(startX, startY);

	// �S�[���ݒ�
	aStar.SetGoal(9, 9);

	// �����̃V�[�h��ݒ�
	srand(static_cast<unsigned>(time(nullptr)));  

	// ��Q���ݒ�
	for(int i = 0; i < 25; i++)
	{
		int x = rand() % width;
		int y = rand() % height;

		if ((x != startX || y != startY) && (x != goalX || y != goalY))
		{
			aStar.SetObstacle(x, y);
		}
	}

	// �o�H�T��
	vector<Node*> path = aStar.FindPath();

	// �p�X������������
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

	cout << "\n" << "��:�ʘH  ��:��Q��  �Z:�ŒZ�o�H" << endl;

	cout << "\n" <<  "�`Map�`" << endl;

	aStar.PrintMap();

	return 0;


}