#ifndef _MY_LINK_MAP_HPP_
#define _MY_LINK_MAP_HPP_
#include <time.h>
#include <vector>
#include <algorithm>
#include <iostream>
#include <boost/algorithm/string.hpp>

#include <fstream>
#include <Eigen/Dense>
using namespace std;
using namespace Eigen;
class LinkMapNode
{
public:
    int index = -1;
    int floorNum;
    int type;
    Eigen::Isometry3d pose;
    bool inited = false;
    float cost = 0;
    LinkMapNode *lastNode = NULL;
    int lastmove = 0;
    string name;
    vector<LinkMapNode *> neighbors;
    vector<bool> checkMark;
    vector<int> movetype;
    vector<float> costmap;
    bool checked()
    {
        bool flag = true;
        for (int i = 0; i < checkMark.size(); i++)
        {
            flag = flag && checkMark[i];
        }
        return flag;
    }
};

class LinkMap
{
private:
    static bool cmpNodeIndex(LinkMapNode a, LinkMapNode b)
    {
        return a.index < b.index;
    }

public:
    vector<LinkMapNode> mapNodeVec;
    vector<LinkMapNode *> goodNodeVec;
    bool read(string nodefile, string linkfile)
    {
        ifstream ifs;
        ifs.open(nodefile.c_str());
        int max_index = 0;
        vector<LinkMapNode> preVec;
        if (!ifs.is_open())
        {
            return false;
        }
        else
        {
            char buffer[1024];
            while (ifs.good() && !ifs.eof())
            {
                ifs.getline(buffer, 1024);
                if (buffer[0] == '#')
                {
                    continue;
                }
                string record(buffer);
                vector<string> fields;
                boost::split(fields, record, boost::is_any_of(";"), boost::token_compress_on);
                if (fields.size() < 11)
                {
                    continue;
                }
                LinkMapNode newnode;
                newnode.index = atoi(fields[0].c_str());
                newnode.name = fields[1];
                newnode.type = atoi(fields[2].c_str());
                newnode.floorNum = atoi(fields[3].c_str());
                Eigen::Vector3d pos(atof(fields[4].c_str()),atof(fields[5].c_str()),atof(fields[6].c_str()));
                Eigen::Quaterniond qua(atof(fields[7].c_str()),atof(fields[8].c_str()),atof(fields[9].c_str()),atof(fields[10].c_str()));
                newnode.pose = Eigen::Isometry3d::Identity();
                newnode.pose.pretranslate(pos);
                newnode.pose.prerotate(qua);
                newnode.inited = true;
                preVec.push_back(newnode);

                if (newnode.index > max_index)
                {
                    max_index = newnode.index;
                }
            }
            ifs.close();
        }
        std::sort(preVec.begin(), preVec.end(), cmpNodeIndex);
        mapNodeVec = vector<LinkMapNode>(max_index + 1);
        for (int i = 0; i < preVec.size(); i++)
        {
            int preIndex = preVec[i].index;
            if (preIndex < 0 || preIndex > max_index)
            {
                continue;
            }
            // cout << "goode" << endl;
            mapNodeVec[preIndex] = preVec[i];
            goodNodeVec.push_back(&(mapNodeVec[preIndex]));
        }
        ifs.open(linkfile.c_str());
        if (!ifs.is_open())
        {
        }
        else
        {
            char buffer[1024];
            while (ifs.good() && !ifs.eof())
            {
                ifs.getline(buffer, 1024);
                if (buffer[0] == '#')
                {

                    continue;
                }
                string record(buffer);
                vector<string> fields;
                boost::split(fields, record, boost::is_any_of(";"), boost::token_compress_on);
                // cout << "有数据："<<fields.size() << endl;
                if (fields.size() < 4)
                {
                    continue;
                }
                int start = atoi(fields[0].c_str());
                int end = atoi(fields[1].c_str());
                int movetype = atoi(fields[2].c_str());
                float cost = atof(fields[3].c_str());
                mapNodeVec[start].neighbors.push_back(&(mapNodeVec[end]));
                mapNodeVec[start].movetype.push_back(movetype);
                mapNodeVec[start].costmap.push_back(cost);
                mapNodeVec[start].checkMark.push_back(false);

                int movetypeinverse = movetype;
                if (movetype == 1)
                {
                    movetypeinverse = 2;
                }
                else if (movetype == 2)
                {
                    movetypeinverse = 1;
                }
                mapNodeVec[end].neighbors.push_back(&(mapNodeVec[start]));
                mapNodeVec[end].movetype.push_back(movetypeinverse);
                mapNodeVec[end].costmap.push_back(cost);
                mapNodeVec[end].checkMark.push_back(false);
            }
            ifs.close();
        }
        return true;
    }
    bool searchPath(LinkMapNode *start, LinkMapNode *end,double& navcost, vector<LinkMapNode *> &nodePath, vector<int> &methodPath)
    {
        navcost=0;
        if (start == NULL || end == NULL)
        {
            return false;
        }
        if (start == end)
        {
            nodePath.push_back(start);
            return true;
        }
        vector<LinkMapNode *> check_stack;
        vector<LinkMapNode *> check_stack_bank;
        vector<LinkMapNode *> resetVec;
        start->lastNode = NULL;
        check_stack.push_back(start);
        resetVec.push_back(start);
        bool finded = false;
        while ((!(check_stack.empty())))
        {
            for (int i = 0; i < check_stack.size(); i++)
            {
                LinkMapNode *pnode = check_stack[i];
                for (int k = 0; k < pnode->neighbors.size(); k++)
                {
                    LinkMapNode *nownode = pnode->neighbors[k];
                    double nowcost = pnode->cost + pnode->costmap[k];
                    // cout << "对比: " << nownode->cost<<"--"<<nowcost<< endl;
                    if (pnode->lastNode == nownode)
                    {
                        continue;
                    }
                    if (!pnode->checkMark[k])
                    {
                        pnode->checkMark[k] = true;
                        if (nowcost < nownode->cost || nownode->cost == 0)
                        {
                            nownode->lastNode = pnode;
                            nownode->lastmove = pnode->movetype[k];
                            nownode->cost = nowcost;
                        }
                    }
                    if (nownode == end)
                    {
                        finded = true;
                    }
                    else if (!nownode->checked())
                    {
                        check_stack_bank.push_back(nownode);
                        resetVec.push_back(nownode);
                    }
                }
            }
            check_stack.clear();
            //清空
            for (int i = 0; i < check_stack_bank.size(); i++) //将暂存的要检查的节点转移到check_stack
            {
                check_stack.push_back(check_stack_bank[i]);
            }
            check_stack_bank.clear(); //清空暂存
        }
        if (finded)
        {
            navcost=end->cost;
            LinkMapNode *node = end;
            while (node->lastNode)
            {
                // cout << "index:" <<node->index<< endl;
                nodePath.push_back(node);
                methodPath.push_back(node->lastmove);
                node = node->lastNode;
            }
            nodePath.push_back(node);
            methodPath.push_back(0);
            std::reverse(nodePath.begin(), nodePath.end());
            std::reverse(methodPath.begin(), methodPath.end());
        }
        for (int i = 0; i < resetVec.size(); i++)
        {
            resetVec[i]->lastNode = NULL;
            resetVec[i]->checkMark = vector<bool>(resetVec[i]->checkMark.size(), false);
            resetVec[i]->cost = 0;
            resetVec[i]->lastmove = 0;
        }
        return finded;
    }
    vector<LinkMapNode *> findByName(string name)
    {
        vector<LinkMapNode *> result;
        for (int i = 0; i < mapNodeVec.size(); i++)
        {
            if (mapNodeVec[i].name == name)
            {
                result.push_back(&(mapNodeVec[i]));
            }
        }
        return result;
    }
};

#endif //_MY_LINK_MAP_HPP_