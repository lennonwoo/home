# -*- coding: utf-8 -*-
class ConfigDict:
    distance = {
        'people': 1.1,
        'face': 0.5,

        'wanglaoji': 0.55,
        'fenda': 0.55,
        'kele': 0.55,
        'xuebi': 0.55,
        'shui': 0.55,
        'hongcha': 0.55,
        'shupian': 0.55,
    }

    box_threshold = {
        'people': 0.7,
        'face': 0.9,

        # 'wanglaoji': 0.6,
        # 'fenda': 0.6,
        # 'kele': 0.6,
        # 'xuebi': 0.6,
        # 'shui': 0.6,
        # 'hongcha': 0.6,
        # 'shupian': 0.6,

        "milk": 0.6,
        "water": 0.6,
        "chips": 0.6,
        "porridge": 0.6,
        "safeguard": 0.6,
        "napkin": 0.6,
        "sprite": 0.6,
        "laoganma": 0.6,
        "cola": 0.6,
        "icetea": 0.6,

        "牛奶": 0.6,
        "矿泉水": 0.6,
        "薯片": 0.6,
        "八宝粥": 0.6,
        "舒服佳": 0.6,
        "抽纸": 0.6,
        "雪碧": 0.6,
        "老干妈": 0.6,
        "可乐": 0.6,
        "冰红茶": 0.6,
    }

    names_cn = [
        "丹尼尔",
        "露丝",
        "迈克尔",
        "约翰",
        "杰克",
        "玛丽",
        "费舍尔",
        "亚当",
        "凯文",
        "汤姆",
    ]

    names_en = [
        "Daniel",
        "Rose",
        "Michael",
        "John",
        "Jack",
        "Mary",
        "Fisher",
        "Adam",
        "Kevin",
        "Tom",
    ]

    objs_cn = [
        "牛奶",
        "矿泉水",
        "薯片",
        "八宝粥",
        "舒服佳",
        "抽纸",
        "雪碧",
        "老干妈",
        "可乐",
        "冰红茶",
    ]

    objs_en = [
        "milk",
        "water",
        "chips",
        "porridge",
        "safeguard",
        "napkin",
        "sprite",
        "laoganma",
        "cola",
        "icetea",
    ]

    arm_grasp_obj_list = [
        "牛奶",
        "矿泉水",
        "薯片",
        "八宝粥",
        "舒服佳",
        "抽纸",
        "雪碧",
        "老干妈",
        "可乐",
        "冰红茶",
        "kele",  # for test
    ]

    @staticmethod
    def get_trans_dict(keys, values):
        dic = {}
        for k, v in zip(keys, values):
            dic[k] = v
        return dic

    @staticmethod
    def get_obj_en2cn():
        return ConfigDict.get_trans_dict(ConfigDict.objs_en, ConfigDict.objs_cn)

    @staticmethod
    def get_obj_cn2en():
        return ConfigDict.get_trans_dict(ConfigDict.objs_cn, ConfigDict.objs_en)

    @staticmethod
    def get_name_en2cn():
        return ConfigDict.get_trans_dict(ConfigDict.names_en, ConfigDict.names_cn)

    @staticmethod
    def get_name_cn2en():
        return ConfigDict.get_trans_dict(ConfigDict.names_cn, ConfigDict.names_en)
