#include "AStar.h"

int main()
{
	//�}�b�v�̕��ݒ�
	const int width = 10; 

	//�}�b�v�̍����ݒ�
	const int height = 10;

	// �C���X�^���X�쐬
	AStar aStar(width, height);

	// �X�^�[�g�ݒ�
	aStar.SetStart(0, 0);

	// �S�[���ݒ�
	aStar.SetGoal(9, 9);

	// ��Q���ݒ�
	aStar.SetObstacle(2, 2);
	aStar.SetObstacle(3, 3);
	aStar.SetObstacle(4, 4);

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

	return 0;


}