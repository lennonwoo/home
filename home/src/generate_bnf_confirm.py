# -*- coding: utf-8 -*-
from config import Config


bnf_format = """\
#BNF+IAT 1.0 UTF-8;
!grammar mission;

!slot <intro>;
!slot <guestname>;

!slot <take>;
!slot <item>;

!slot <yes>;
!slot <no>;

!start <missionstart>;
<missionstart>:<task1>|<task2>;

<task1>:<mission0><mission1>;
<task2>:<mission2>;


<mission0>:<intro><guestname>;
<intro>:我是|我的名字是|请叫我;
<guestname>:%s;


<mission1>:<take><item>;
<take>:请帮我拿|帮我拿|给我拿|去拿|我要|给我拿|我需要|我想要;
<item>:%s;

<mission2>:<yes>|<no>;
<yes>:正确;
<no>:错误;
"""


def main():
    names_lst = Config.names_cn
    objs_lst = Config.objs_cn
    path = Config.base_path

    with open(path + "home_generated_confirmed.bnf", "w+") as f:
        bnf_content = bnf_format % ("|".join(names_lst),
                                    "|".join(objs_lst))
        f.write(bnf_content)


if __name__ == '__main__':
    main()
